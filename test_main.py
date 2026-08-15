import io
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from main import RobotApp, State
from robot_agent import ActionResult, MotionCall, TargetMission, TargetObservation, TurnOutcome


class AppPackagingTests(unittest.TestCase):
    def test_app_does_not_reinstall_app_lab_base_opencv(self):
        requirements = Path("python/requirements.txt").read_text(encoding="utf-8").lower()
        self.assertNotIn("opencv", requirements)

    def test_sketch_matches_physical_motor_channel_table(self):
        sketch = Path("sketch/sketch.ino").read_text(encoding="utf-8")
        expected_lines = (
            "static const int FRONT_LEFT_POLARITY = -1;",
            "static const int FRONT_RIGHT_POLARITY = 1;",
            "static const int REAR_LEFT_POLARITY = 1;",
            "static const int REAR_RIGHT_POLARITY = -1;",
            "setMotor(D1_BIN1, D1_BIN2, D1_PWMB, direction * REAR_RIGHT_POLARITY, pwm);",
            "setMotor(D2_AIN1, D2_AIN2, D2_PWMA, direction * FRONT_RIGHT_POLARITY, pwm);",
            "setMotor(D2_BIN1, D2_BIN2, D2_PWMB, direction * REAR_LEFT_POLARITY, pwm);",
            "setMotor(D2_BIN1, D2_BIN2, D2_PWMB, left * REAR_LEFT_POLARITY, pwm);",
            "setMotor(D2_AIN1, D2_AIN2, D2_PWMA, right * FRONT_RIGHT_POLARITY, pwm);",
            "setMotor(D1_BIN1, D1_BIN2, D1_PWMB, right * REAR_RIGHT_POLARITY, pwm);",
        )
        for line in expected_lines:
            with self.subTest(line=line):
                self.assertIn(line, sketch)


class FakeAgent:
    robot_name = "Scout"

    def __init__(self):
        self.synthesized = []
        self.camera_index = 0
        self.turns = []

    def synthesize(self, text):
        self.synthesized.append(text)
        return io.BytesIO(b"fake pcm")

    def transcribe(self, _wav):
        return "test phrase"

    def run_turn(self, transcript, _on_action, is_cancelled):
        self.turns.append((transcript, is_cancelled()))
        return TurnOutcome()

    def close(self):
        pass

    def set_camera_index(self, index):
        self.camera_index = index

    def available_cameras(self):
        return [0, 2]

    def capture_frame(self):
        return "data:image/jpeg;base64,abc"


class FakeAudio:
    def __init__(self):
        self.played = []
        self.input_device = None
        self.output_device = None
        self.speech_rms_threshold = 900

    def play(self, stream):
        self.played.append(stream.read())

    def set_input_device(self, value):
        self.input_device = value
        return "Test microphone"

    def set_output_device(self, value):
        self.output_device = value
        return "Test output"

    def describe_devices(self):
        return "Test microphone", "Test output"

    def test_input(self, seconds):
        return {"voiced_percent": 50, "rms": 100, "peak": 500}, b"test wav"

    def calibrate_input(self, seconds=1.5):
        self.speech_rms_threshold = 1000
        return self.speech_rms_threshold

    @staticmethod
    def list_devices():
        print("fake devices")


class RobotAppActionTests(unittest.TestCase):
    def test_speech_start_does_not_launch_vision_processing(self):
        agent = FakeAgent()
        app = RobotApp(agent, FakeAudio())
        app._on_speech_start()
        self.assertEqual(app.capture_generation, 1)
        self.assertEqual(agent.turns, [])

    def test_completed_utterance_sends_transcript_without_eager_vision(self):
        agent = FakeAgent()
        app = RobotApp(agent, FakeAudio())
        app.speech_generation = 1
        app.turn_generation = 1
        app._process_turn(b"test wav")
        self.assertEqual(agent.turns, [("test phrase", False)])

    def test_interrupting_utterance_does_not_add_spoken_chatter(self):
        agent = FakeAgent()
        audio = FakeAudio()
        app = RobotApp(agent, audio)
        app.speech_generation = 1
        app.turn_generation = 1
        app._process_turn(b"test wav", interrupted=True)
        self.assertEqual(agent.synthesized, [])
        self.assertEqual(audio.played, [])
        self.assertEqual(agent.turns, [("test phrase", False)])

    def test_action_logs_proposal_before_silent_execution(self):
        app = RobotApp(FakeAgent(), FakeAudio())
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("move", 30, 8))
        lines = output.getvalue().splitlines()
        proposed = next(i for i, line in enumerate(lines) if line.startswith("[PROPOSED]"))
        acting = next(i for i, line in enumerate(lines) if line == "[STATE] acting")
        executed = next(i for i, line in enumerate(lines) if line.startswith("[TOOL]"))
        self.assertLess(proposed, acting)
        self.assertLess(acting, executed)
        self.assertIsNotNone(result.content)
        self.assertEqual(app.state, State.PROCESSING)

    def test_motion_is_silent(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("turn", 50, -90))
        self.assertEqual(audio.played, [])
        self.assertIsNone(result.interruption)
        self.assertIn("[TOOL]", output.getvalue())
        self.assertEqual(app.state, State.PROCESSING)

    def test_new_speech_cancels_pending_action(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)
        app.worker_busy.set()
        app._on_speech_start()
        self.assertIn(app.capture_generation, app.interrupt_generations)
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("move", 30, 8))
        self.assertEqual(result.interruption, b"")
        self.assertEqual(audio.played, [])
        self.assertIn("[CANCELLED]", output.getvalue())
        self.assertNotIn("[TOOL]", output.getvalue())

    def test_microphone_capture_is_paused_while_motors_run(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)

        def observe_motion(_call, _stop):
            self.assertFalse(app.can_listen.is_set())
            self.assertTrue(app.capture_stop.is_set())
            return '{"status":"completed"}'

        with patch("main.execute_motion", side_effect=observe_motion):
            result = app._handle_action(MotionCall("turn", 40, 90))
        self.assertTrue(app.can_listen.is_set())
        self.assertEqual(result.content, '{"status":"completed"}')

    def test_target_mission_continues_search_align_and_approach_until_reached(self):
        agent = FakeAgent()
        observations = iter(
            [
                TargetObservation(False),
                TargetObservation(False),
                TargetObservation(True, "left", "small", False),
                TargetObservation(True, "center", "medium", False),
                TargetObservation(True, "center", "large", True),
            ]
        )
        agent.locate_target = lambda *_args: next(observations)
        app = RobotApp(agent, FakeAudio())
        spoken = []
        motions = []
        app._speak = spoken.append

        def complete(call):
            motions.append(call)
            return ActionResult(content='{"status":"completed"}')

        app._handle_action = complete
        app._run_target_mission(TargetMission("shoe", "toward"))
        self.assertEqual(
            motions,
            [
                MotionCall("spin", 20, 0.5),
                MotionCall("spin", 20, 0.5),
                MotionCall("spin", 20, 0.2),
                MotionCall("move", 20, 0.75),
            ],
        )
        self.assertEqual(
            spoken,
            ["Searching for shoe.", "I found the shoe. Moving toward it.", "I reached the shoe."],
        )

    def test_microphone_processing_is_paused_only_for_playback(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)

        def observe_playback(stream):
            self.assertFalse(app.can_listen.is_set())
            self.assertTrue(app.capture_stop.is_set())
            audio.played.append(stream.read())

        audio.play = observe_playback
        app._speak("Hello")
        self.assertTrue(app.can_listen.is_set())
        self.assertEqual(audio.played, [b"fake pcm"])

    def test_runtime_device_commands_do_not_change_env_defaults(self):
        agent = FakeAgent()
        audio = FakeAudio()
        app = RobotApp(agent, audio)
        output = io.StringIO()
        with redirect_stdout(output):
            app._handle_command("/mic 4")
            app._handle_command("/output 9")
            app._handle_command("/camera 2")
            app._handle_command("/test-mic 1")
            app._handle_command("/test-camera")
            app._handle_command("/status")
        self.assertEqual(audio.input_device, "4")
        self.assertEqual(audio.output_device, "9")
        self.assertEqual(agent.camera_index, 2)
        self.assertIn("camera test passed", output.getvalue())
        self.assertIn("voiced=50.0%", output.getvalue())
        self.assertIn("mic transcription: test phrase", output.getvalue())

    def test_quit_command_stops_app(self):
        app = RobotApp(FakeAgent(), FakeAudio())
        app._handle_command("/quit")
        self.assertFalse(app.running)
        self.assertTrue(app.capture_stop.is_set())

    def test_unavailable_device_does_not_crash_command_loop(self):
        audio = FakeAudio()

        def unavailable(_value):
            raise OSError("device busy")

        audio.set_input_device = unavailable
        app = RobotApp(FakeAgent(), audio)
        output = io.StringIO()
        with redirect_stdout(output):
            app._handle_command("/mic 4")
        self.assertTrue(app.running)
        self.assertIn("command failed: device busy", output.getvalue())


if __name__ == "__main__":
    unittest.main()
