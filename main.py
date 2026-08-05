"""Scout voice-and-vision robot simulator entry point."""

from __future__ import annotations

import argparse
import json
import os
import queue
import sys
import threading
from dataclasses import dataclass
from enum import Enum

from audio_io import VoiceIO
from robot_agent import (
    ActionResult,
    GroqRobot,
    MotionCall,
    announcement,
    cancelled_line,
    execute_motion,
    proposed_line,
)


class State(Enum):
    LISTENING = "listening"
    RECORDING = "recording"
    PROCESSING = "processing"
    ANNOUNCING = "announcing"
    ACTING = "acting"
    SPEAKING = "speaking"


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
    camera_index: int = 0


class RobotApp:
    def __init__(self, agent: GroqRobot, audio: VoiceIO) -> None:
        self.agent = agent
        self.audio = audio
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
        self.worker: threading.Thread | None = None

    def set_state(self, state: State) -> None:
        if state != self.state:
            self.state = state
            print(f"[STATE] {state.value}")

    def run(self) -> None:
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
                    print("[PROCESSING] discarded superseded utterance")
                    continue
                self.turn_generation = generation
                self.motion_stop.clear()
            self.worker_busy.set()
            try:
                self._process_turn(wav)
            finally:
                self.worker_busy.clear()

    def _process_turn(self, wav: bytes) -> None:
        self.set_state(State.PROCESSING)
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
        print(f"You: {transcript}")

        try:
            outcome = self.agent.run_turn(
                transcript,
                self._handle_action,
                self._turn_cancelled,
            )
        except Exception as exc:
            print(f"[LLM] request failed: {exc}")
            if not self._turn_cancelled():
                self._speak("I couldn't process that. Please try again.")
            return
        if self._turn_cancelled():
            print("[PROCESSING] stale response discarded")
        elif outcome.reply:
            self._speak(outcome.reply)

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
        if self._turn_cancelled():
            print(cancelled_line(call))
            return ActionResult(interruption=b"")
        text = announcement(call)
        print(f"{self.agent.robot_name}: {text}")
        try:
            audio = self.agent.synthesize(text)
        except Exception as exc:
            print(f"[TTS] announcement failed; motion cancelled: {exc}")
            print(cancelled_line(call).replace("user interruption", "speech synthesis failure"))
            return ActionResult(content=json.dumps({"error": "announcement could not be spoken"}))
        if self._turn_cancelled():
            audio.close()
            print(cancelled_line(call))
            return ActionResult(interruption=b"")

        try:
            self._play_audio(audio, State.ANNOUNCING)
        except Exception as exc:
            print(f"[AUDIO] playback failed; motion cancelled: {exc}")
            return ActionResult(content=json.dumps({"error": "announcement playback failed"}))
        if self._turn_cancelled():
            print(cancelled_line(call))
            return ActionResult(interruption=b"")
        if self.worker is not None and not self.listener_ready.wait(timeout=2):
            print(cancelled_line(call).replace("user interruption", "microphone unavailable"))
            return ActionResult(content=json.dumps({"error": "microphone unavailable"}))

        self.set_state(State.ACTING)
        content = execute_motion(call, self.motion_stop)
        self.set_state(State.PROCESSING)
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
            self.can_listen.set()

    def close(self) -> None:
        self.running = False
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
        camera_index=camera_index,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Voice-and-vision robot simulator")
    parser.add_argument("--list-devices", action="store_true", help="list audio devices and exit")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list_devices:
        try:
            VoiceIO.list_devices()
            return 0
        except RuntimeError as exc:
            print(f"Error: {exc}")
            return 1
    try:
        settings = load_settings()
        audio = VoiceIO(settings.mic_device, settings.output_device)
        agent = GroqRobot(
            settings.api_key,
            robot_name=settings.robot_name,
            llm_model=settings.llm_model,
            vision_model=settings.vision_model,
            stt_model=settings.stt_model,
            deepgram_api_key=settings.deepgram_api_key,
            tts_model=settings.tts_model,
            camera_index=settings.camera_index,
        )
        app = RobotApp(agent, audio)
    except RuntimeError as exc:
        print(f"Error: {exc}")
        return 1
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nStopping Scout.")
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
