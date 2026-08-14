"""Optimus UNO Q voice-and-vision robot controller entry point."""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum

from audio_io import VoiceIO
from live_view import LiveCameraView
from robot_agent import (
    ActionResult,
    GroqRobot,
    MotionCall,
    TargetMission,
    cancelled_line,
    parse_direct_motion,
    parse_target_mission,
    proposed_line,
)

# Arduino App Lab provides the managed lifecycle and MCU Bridge on UNO Q.
try:
    from arduino.app_utils import App as ArduinoApp
except ImportError:
    ArduinoApp = None


try:
    from execute_motion_bridge import execute_motion, read_sensors, stop_motors, wait_for_mcu

    HARDWARE_MOTION = True
    print("[BOOT] Arduino App Lab Bridge detected - physical motor mode")
except ImportError as exc:
    if ArduinoApp is not None:
        raise RuntimeError(
            "Arduino App Lab is present but the physical motion backend could not load"
        ) from exc
    from robot_agent import execute_motion

    HARDWARE_MOTION = False

    def wait_for_mcu() -> dict[str, object]:
        return {"ready": False, "status": "simulated"}

    def stop_motors(_reason: str = "simulator") -> None:
        return None

    def read_sensors() -> dict[str, object]:
        return {"status": "unavailable", "reason": "not running on UNO Q"}

    print("[BOOT] Arduino Bridge unavailable - simulated motion mode")


class State(Enum):
    LISTENING = "listening"
    RECORDING = "recording"
    PROCESSING = "processing"
    ANNOUNCING = "announcing"
    ACTING = "acting"
    SPEAKING = "speaking"


POST_PLAYBACK_MIC_COOLDOWN_SECONDS = 0.20
POST_MOTION_MIC_COOLDOWN_SECONDS = 0.15
MISSION_TIMEOUT_SECONDS = 1200.0
MAX_MISSION_ACTIONS = 240
SEARCH_SPIN_SECONDS = 0.50
REACQUIRE_SPIN_SECONDS = 0.20
ALIGN_SPIN_SECONDS = 0.20
APPROACH_STEP_SECONDS = 0.75
FINAL_APPROACH_STEP_SECONDS = 0.35
AWAY_TURN_SECONDS = 2.0
AWAY_DRIVE_SECONDS = 1.5


@dataclass(frozen=True)
class Settings:
    api_key: str
    deepgram_api_key: str
    robot_name: str = "Scout"
    llm_model: str = "openai/gpt-oss-120b"
    vision_model: str = "qwen/qwen3.6-27b"
    stt_model: str = "whisper-large-v3-turbo"
    tts_model: str = "aura-2-thalia-en"
    mic_device: str | None = None
    output_device: str | None = None
    playback_gain: float = 3.0
    camera_index: int = 0


class RobotApp:
    def __init__(
        self,
        agent: GroqRobot,
        audio: VoiceIO,
        live_view: LiveCameraView | None = None,
    ) -> None:
        self.agent = agent
        self.audio = audio
        self.live_view = live_view
        self.state: State | None = None
        self.running = True
        self.command_queue: queue.Queue[str] = queue.Queue()
        self.turn_queue: queue.Queue[tuple[bytes, int] | None] = queue.Queue()
        self.capture_stop = threading.Event()
        self.can_listen = threading.Event()
        self.can_listen.set()
        self.listener_ready = threading.Event()
        self.audio_lock = threading.Lock()
        self.worker_busy = threading.Event()
        self.motion_stop = threading.Event()
        self.generation_lock = threading.Lock()
        self.speech_generation = 0
        self.capture_generation = 0
        self.turn_generation = 0
        self.interrupt_generations: set[int] = set()
        self.worker: threading.Thread | None = None

    def set_state(self, state: State) -> None:
        if state != self.state:
            self.state = state
            print(f"[STATE] {state.value}")
            if self.live_view is not None:
                self.live_view.publish_status(state.value)

    def run(self) -> None:
        if HARDWARE_MOTION:
            self.set_state(State.PROCESSING)
            wait_for_mcu()
        if self.live_view is not None:
            self.live_view.set_emergency_stop(self._emergency_stop)
            self.live_view.start()
        print("Microphone processing is disabled while Scout speaks.")
        print("Type /help for live device commands. Press Ctrl+C to stop.\n")
        self._calibrate_mic()
        threading.Thread(target=self._read_commands, name="console-commands", daemon=True).start()
        self.worker = threading.Thread(target=self._process_turns, name="agent-worker", daemon=True)
        self.worker.start()
        while self.running:
            self._drain_commands()
            if not self.running:
                break
            if not self.can_listen.wait(timeout=0.1):
                continue
            with self.audio_lock:
                self.capture_stop.clear()
                if not self.can_listen.is_set() or not self.command_queue.empty():
                    continue
                if not self.worker_busy.is_set():
                    self.set_state(State.LISTENING)
                self.listener_ready.clear()
                wav = self.audio.capture(
                    on_ready=self.listener_ready.set,
                    on_speech_start=self._on_speech_start,
                    stop_event=self.capture_stop,
                )
                self.listener_ready.clear()
            if self._drain_commands():
                wav = None
            if not self.running:
                break
            if wav is None:
                continue
            self.turn_queue.put((wav, self.capture_generation))

    def _on_speech_start(self) -> None:
        with self.generation_lock:
            self.speech_generation += 1
            self.capture_generation = self.speech_generation
            if self.worker_busy.is_set():
                self.interrupt_generations.add(self.capture_generation)
        self.motion_stop.set()
        if self.worker_busy.is_set():
            print("[INTERRUPT] speech detected; cancelling the active turn and motion")
        self.set_state(State.RECORDING)

    def _turn_cancelled(self) -> bool:
        with self.generation_lock:
            return self.speech_generation > self.turn_generation

    def _process_turns(self) -> None:
        while self.running:
            item = self.turn_queue.get()
            if item is None:
                return
            wav, generation = item
            with self.generation_lock:
                if generation < self.speech_generation:
                    self.interrupt_generations.discard(generation)
                    print("[PROCESSING] discarded superseded utterance")
                    continue
                self.turn_generation = generation
                interrupted = generation in self.interrupt_generations
                self.interrupt_generations.discard(generation)
                self.motion_stop.clear()
            self.worker_busy.set()
            try:
                self._process_turn(wav, interrupted)
            finally:
                self.worker_busy.clear()

    def _process_turn(self, wav: bytes, interrupted: bool = False) -> None:
        if interrupted:
            print("[INTERRUPT] previous task stopped; processing the new command")
            if self._turn_cancelled():
                return
        self.set_state(State.PROCESSING)
        turn_started = time.monotonic()
        try:
            transcript = self.agent.transcribe(wav)
        except Exception as exc:
            print(f"[STT] failed; listening again: {exc}")
            return
        if self._turn_cancelled():
            print("[PROCESSING] turn superseded by new speech")
            return
        if not transcript:
            print("[STT] no speech recognized")
            return
        print(f"[PERF] speech-to-text: {time.monotonic() - turn_started:.2f}s")
        print(f"You: {transcript}")
        if self.live_view is not None:
            self.live_view.publish_status("planning", transcript)

        mission = parse_target_mission(transcript)
        if mission is not None:
            self._run_target_mission(mission)
            return

        try:
            direct_motion = parse_direct_motion(transcript)
        except ValueError as exc:
            print(f"[COMMAND] invalid direct motion: {exc}")
            self._speak(f"That motion value is invalid: {exc}.")
            return
        if direct_motion is not None:
            self._handle_action(direct_motion)
            return

        try:
            planning_started = time.monotonic()
            outcome = self.agent.run_turn(
                transcript,
                self._handle_action,
                self._turn_cancelled,
            )
            print(f"[PERF] agent response: {time.monotonic() - planning_started:.2f}s")
        except Exception as exc:
            print(f"[LLM] request failed: {exc}")
            if not self._turn_cancelled():
                self._speak("I couldn't process that. Please try again.")
            return
        if self._turn_cancelled():
            print("[PROCESSING] stale response discarded")
        elif outcome.reply:
            self._speak(outcome.reply)

    def _run_target_mission(self, mission: TargetMission) -> None:
        target = mission.target
        direction_word = "toward" if mission.behavior == "toward" else "away from"
        self._speak(f"Searching for {target}.")
        if self._turn_cancelled():
            return

        found = False
        last_position = "unknown"
        missing_after_found = 0
        search_turns = 0
        actions = 0
        deadline = time.monotonic() + MISSION_TIMEOUT_SECONDS

        while (
            self.running
            and not self._turn_cancelled()
            and not self.motion_stop.is_set()
            and actions < MAX_MISSION_ACTIONS
            and time.monotonic() < deadline
        ):
            self.set_state(State.PROCESSING)
            try:
                observation = self.agent.locate_target(target, self._turn_cancelled)
            except InterruptedError:
                return
            except Exception as exc:
                print(f"[TARGET] inspection failed: {exc}")
                self._speak("The camera failed. I stopped the mission.")
                return

            if self.live_view is not None:
                self.live_view.publish_status(
                    "target_tracking",
                    (
                        f"{target}: visible={observation.visible}, "
                        f"position={observation.position}, scale={observation.scale}"
                    ),
                )

            if not observation.visible:
                missing_after_found += 1 if found else 0
                if found and missing_after_found <= 4 and last_position in {"left", "right"}:
                    amount = REACQUIRE_SPIN_SECONDS if last_position == "left" else -REACQUIRE_SPIN_SECONDS
                else:
                    amount = SEARCH_SPIN_SECONDS
                    search_turns += 1
                result = self._handle_action(MotionCall("spin", 20, amount))
                actions += 1
                if not self._action_completed(result):
                    return
                # A complete sweep with no sighting changes the viewpoint before scanning again.
                if not found and search_turns >= 12:
                    search_turns = 0
                    result = self._handle_action(MotionCall("move", 20, APPROACH_STEP_SECONDS))
                    actions += 1
                    if not self._action_completed(result):
                        return
                continue

            missing_after_found = 0
            last_position = observation.position
            if not found:
                found = True
                self._speak(f"I found the {target}. Moving {direction_word} it.")
                if self._turn_cancelled():
                    return

            if mission.behavior == "away":
                if observation.position in {"left", "right"}:
                    amount = ALIGN_SPIN_SECONDS if observation.position == "left" else -ALIGN_SPIN_SECONDS
                    result = self._handle_action(MotionCall("spin", 20, amount))
                    actions += 1
                    if not self._action_completed(result):
                        return
                    continue
                for call in (
                    MotionCall("spin", 20, AWAY_TURN_SECONDS),
                    MotionCall("move", 20, AWAY_DRIVE_SECONDS),
                ):
                    result = self._handle_action(call)
                    actions += 1
                    if not self._action_completed(result):
                        return
                self._speak(f"I moved away from the {target}.")
                return

            if observation.reached:
                self._speak(f"I reached the {target}.")
                return
            if observation.position in {"left", "right"}:
                amount = ALIGN_SPIN_SECONDS if observation.position == "left" else -ALIGN_SPIN_SECONDS
                result = self._handle_action(MotionCall("spin", 20, amount))
            else:
                step = FINAL_APPROACH_STEP_SECONDS if observation.scale == "large" else APPROACH_STEP_SECONDS
                result = self._handle_action(MotionCall("move", 20, step))
            actions += 1
            if not self._action_completed(result):
                return

        if self.motion_stop.is_set() or self._turn_cancelled():
            print("[TARGET] mission interrupted")
            return
        print("[TARGET] mission containment deadline reached")
        self._speak(f"I could not reach the {target} before the mission timeout.")

    @staticmethod
    def _action_completed(result: ActionResult) -> bool:
        if result.interruption is not None or not result.content:
            return False
        try:
            status = json.loads(result.content).get("status")
        except (json.JSONDecodeError, AttributeError):
            return False
        return status in {"completed", "simulated"}

    def _read_commands(self) -> None:
        for line in sys.stdin:
            command = line.strip()
            if command:
                self.command_queue.put(command)
                self.capture_stop.set()

    def _drain_commands(self) -> bool:
        processed = False
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return processed
            processed = True
            self._handle_command(command)

    def _handle_command(self, line: str) -> None:
        command, _, argument = line.partition(" ")
        command = command.lower()
        argument = argument.strip()
        try:
            if command == "/help":
                print(
                    "Runtime commands:\n"
                    "  /devices             list audio devices and webcam indices\n"
                    "  /status              show active devices\n"
                    "  /mic ID_OR_NAME      switch microphone; use 'default' to reset\n"
                    "  /output ID_OR_NAME   switch speech output\n"
                    "  /camera INDEX        switch webcam\n"
                    "  /test-mic [SECONDS]  measure and transcribe the active mic\n"
                    "  /calibrate-mic       recalibrate the quiet-noise threshold\n"
                    "  /test-camera         capture one frame without sending it\n"
                    "  /sensors             read MCU distance and IMU sensors\n"
                    "  /stop                immediately stop and brake all motors\n"
                    "  /quit                stop the app"
                )
            elif command == "/devices":
                print("Audio devices:")
                self.audio.list_devices()
                cameras = self.agent.available_cameras()
                print(f"Webcam indices: {cameras or 'none found'}")
            elif command == "/status":
                input_name, output_name = self.audio.describe_devices()
                print(f"[CONFIG] microphone: {self.audio.input_device!r} ({input_name})")
                print(f"[CONFIG] speech RMS threshold: {self.audio.speech_rms_threshold:.0f}")
                print(f"[CONFIG] output: {self.audio.output_device!r} ({output_name})")
                print(f"[CONFIG] camera: {self.agent.camera_index}")
                backend = "UNO Q hardware" if HARDWARE_MOTION else "simulator"
                print(f"[CONFIG] motion backend: {backend}")
            elif command == "/mic":
                if not argument:
                    raise ValueError("usage: /mic ID_OR_NAME")
                value = None if argument.lower() == "default" else argument
                name = self.audio.set_input_device(value)
                print(f"[CONFIG] microphone switched to {name}")
                self._calibrate_mic()
            elif command == "/output":
                if not argument:
                    raise ValueError("usage: /output ID_OR_NAME")
                value = None if argument.lower() == "default" else argument
                name = self.audio.set_output_device(value)
                print(f"[CONFIG] output switched to {name}")
            elif command == "/camera":
                if not argument:
                    raise ValueError("usage: /camera INDEX")
                self.agent.set_camera_index(int(argument))
                print(f"[CONFIG] camera switched to index {self.agent.camera_index}")
            elif command == "/test-mic":
                seconds = float(argument) if argument else 3.0
                if not 1 <= seconds <= 10:
                    raise ValueError("microphone test duration must be from 1 to 10 seconds")
                print(f"[CONFIG] testing microphone for {seconds:g}s; speak now...")
                self.set_state(State.RECORDING)
                result, wav = self.audio.test_input(seconds)
                print(
                    f"[CONFIG] mic test: voiced={result['voiced_percent']:.1f}%, "
                    f"RMS={result['rms']:.0f}, peak={result['peak']:.0f}"
                )
                self.set_state(State.PROCESSING)
                transcript = self.agent.transcribe(wav)
                print(f"[CONFIG] mic transcription: {transcript or '(no speech recognized)'}")
            elif command == "/calibrate-mic":
                self._calibrate_mic()
            elif command == "/test-camera":
                self.set_state(State.PROCESSING)
                frame = self.agent.capture_frame()
                if frame is not None:
                    print(f"[CONFIG] camera test passed ({len(frame):,} encoded characters)")
                else:
                    print("[CONFIG] camera test failed")
            elif command == "/sensors":
                print(f"[SENSOR] {json.dumps({'mcu': read_sensors()}, sort_keys=True)}")
            elif command == "/stop":
                self.motion_stop.set()
                stop_motors("console_stop")
            elif command == "/quit":
                self.running = False
                self.capture_stop.set()
                print("Stopping Scout.")
            else:
                print(f"[CONFIG] unknown command: {line!r}; type /help")
        except Exception as exc:
            print(f"[CONFIG] command failed: {exc}")

    def _calibrate_mic(self) -> None:
        print("[AUDIO] calibrating quiet level for 1.5s; please stay silent...")
        self.set_state(State.RECORDING)
        threshold = self.audio.calibrate_input()
        print(f"[AUDIO] speech RMS threshold: {threshold:.0f}")

    def _handle_action(self, call: MotionCall) -> ActionResult:
        print(proposed_line(call))
        if self.live_view is not None:
            self.live_view.publish_status("proposed", proposed_line(call))
        if self._turn_cancelled():
            print(cancelled_line(call))
            return ActionResult(interruption=b"")
        # Motor and gearbox noise repeatedly crossed the microphone threshold in the
        # field log and cancelled every move. Close capture for the brief physical
        # action; the Web UI/app stop remains available throughout.
        self.can_listen.clear()
        self.capture_stop.set()
        self.listener_ready.clear()
        try:
            with self.audio_lock:
                if self.motion_stop.is_set() or self._turn_cancelled():
                    print(cancelled_line(call))
                    return ActionResult(interruption=b"")
                self.set_state(State.ACTING)
                content = execute_motion(call, self.motion_stop)
        finally:
            self.set_state(State.PROCESSING)
            time.sleep(POST_MOTION_MIC_COOLDOWN_SECONDS)
            self.can_listen.set()
        if self.live_view is not None:
            self.live_view.publish_status("motion_result", content)
        if self.motion_stop.is_set() or self._turn_cancelled():
            print(cancelled_line(call))
            return ActionResult(interruption=b"")
        return ActionResult(content=content)

    def _speak(self, text: str) -> None:
        print(f"{self.agent.robot_name}: {text}")
        try:
            audio = self.agent.synthesize(text)
        except Exception as exc:
            print(f"[TTS] failed; response shown in terminal only: {exc}")
            return
        if self._turn_cancelled():
            audio.close()
            return
        try:
            self._play_audio(audio, State.SPEAKING)
        except Exception as exc:
            print(f"[AUDIO] playback failed: {exc}")

    def _emergency_stop(self) -> None:
        self.motion_stop.set()
        stop_motors("web_emergency_stop")
        print("[STOP] emergency stop requested from live camera view")

    def _play_audio(self, audio: object, state: State) -> None:
        self.can_listen.clear()
        self.capture_stop.set()
        self.listener_ready.clear()
        try:
            with self.audio_lock:
                self.set_state(state)
                self.audio.play(audio)
        finally:
            audio.close()
            self.set_state(State.PROCESSING)
            # USB speakers and webcam microphones often have a short acoustic
            # tail. Reopening capture immediately can make the robot hear its
            # own final word and cancel the motion it just announced.
            time.sleep(POST_PLAYBACK_MIC_COOLDOWN_SECONDS)
            self.can_listen.set()

    def close(self) -> None:
        self.running = False
        self.motion_stop.set()
        if HARDWARE_MOTION:
            stop_motors("python_shutdown")
        if self.live_view is not None:
            self.live_view.close()
        self.capture_stop.set()
        self.can_listen.set()
        self.turn_queue.put(None)
        if self.worker is not None:
            self.worker.join(timeout=2)
        self.agent.close()


def load_settings() -> Settings:
    try:
        from dotenv import load_dotenv
    except ImportError as exc:
        raise RuntimeError("python-dotenv is not installed") from exc
    load_dotenv()
    api_key = os.getenv("GROQ_API_KEY", "").strip()
    if not api_key:
        raise RuntimeError("GROQ_API_KEY is missing; copy .env.example to .env and set it")
    deepgram_api_key = os.getenv("DEEPGRAM_API_KEY", "").strip()
    if not deepgram_api_key:
        raise RuntimeError("DEEPGRAM_API_KEY is missing; add it to .env")
    try:
        camera_index = int(os.getenv("CAMERA_INDEX", "0"))
    except ValueError as exc:
        raise RuntimeError("CAMERA_INDEX must be an integer") from exc
    try:
        playback_gain = float(os.getenv("PLAYBACK_GAIN", "3.0"))
    except ValueError as exc:
        raise RuntimeError("PLAYBACK_GAIN must be numeric") from exc
    if not 0.1 <= playback_gain <= 3.0:
        raise RuntimeError("PLAYBACK_GAIN must be between 0.1 and 3.0")
    return Settings(
        api_key=api_key,
        deepgram_api_key=deepgram_api_key,
        robot_name=os.getenv("ROBOT_NAME", "Scout"),
        llm_model=os.getenv("LLM_MODEL", "openai/gpt-oss-120b"),
        vision_model=os.getenv("VISION_MODEL", "qwen/qwen3.6-27b"),
        stt_model=os.getenv("STT_MODEL", "whisper-large-v3-turbo"),
        tts_model=os.getenv("DEEPGRAM_TTS_MODEL", "aura-2-thalia-en"),
        mic_device=os.getenv("MIC_DEVICE"),
        output_device=os.getenv("OUTPUT_DEVICE"),
        playback_gain=playback_gain,
        camera_index=camera_index,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="UNO Q voice-and-vision robot controller")
    parser.add_argument("--list-devices", action="store_true", help="list audio devices and exit")
    return parser.parse_args()


def create_robot_app() -> RobotApp:
    settings = load_settings()
    agent = GroqRobot(
        settings.api_key,
        robot_name=settings.robot_name,
        llm_model=settings.llm_model,
        vision_model=settings.vision_model,
        stt_model=settings.stt_model,
        deepgram_api_key=settings.deepgram_api_key,
        tts_model=settings.tts_model,
        camera_index=settings.camera_index,
        physical_motion=HARDWARE_MOTION,
    )
    live_view = LiveCameraView(agent.camera) if HARDWARE_MOTION else None
    return RobotApp(
        agent,
        VoiceIO(settings.mic_device, settings.output_device, settings.playback_gain),
        live_view,
    )


def run_robot_app() -> int:
    try:
        app = create_robot_app()
    except RuntimeError as exc:
        print(f"Error: {exc}")
        return 1
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nStopping Scout.")
    except Exception as exc:
        print(f"Error: {exc}")
        return 1
    finally:
        app.close()
    return 0


def app_lab_main() -> int:
    """Run the voice controller inside Arduino App Lab's lifecycle."""
    if ArduinoApp is None:
        raise RuntimeError("Arduino App Lab is unavailable")
    try:
        robot_app = create_robot_app()
    except RuntimeError as exc:
        print(f"Error: {exc}")
        return 1
    started = threading.Event()

    def user_loop() -> None:
        # App.run repeatedly invokes user_loop; the robot controller owns its
        # continuous listen/process loop and must only be started once.
        if started.is_set():
            time.sleep(0.1)
            return
        started.set()
        try:
            robot_app.run()
        finally:
            robot_app.close()

    ArduinoApp.run(user_loop=user_loop)
    return 0


def main() -> int:
    args = parse_args()
    if args.list_devices:
        try:
            VoiceIO.list_devices()
            return 0
        except RuntimeError as exc:
            print(f"Error: {exc}")
            return 1
    return run_robot_app()


if __name__ == "__main__":
    raise SystemExit(app_lab_main() if ArduinoApp is not None else main())
