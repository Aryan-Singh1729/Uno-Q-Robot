import io
import threading
import unittest
from concurrent.futures import Future
from contextlib import redirect_stdout

from main import RobotApp, State
from robot_agent import MotionCall, TurnOutcome


class FakeAgent:
    robot_name = "Scout"

    def __init__(self):
        self.synthesized = []
        self.camera_index = 0
        self.vision_started = threading.Event()
        self.turns = []

    def synthesize(self, text):
        self.synthesized.append(text)
        return io.BytesIO(b"fake pcm")

    def transcribe(self, _wav):
        return "test phrase"

    def describe_scene(self):
        self.vision_started.set()
        return "Scene: chair ahead; path clear."

    def run_turn(self, transcript, scene_description, _on_action):
        self.turns.append((transcript, scene_description))
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
    def test_speech_start_launches_vision_processing(self):
        agent = FakeAgent()
        app = RobotApp(agent, FakeAudio())
        app._on_speech_start()
        self.assertTrue(agent.vision_started.wait(timeout=1))
        future = app._take_vision(app.capture_generation)
        self.assertEqual(future.result(timeout=1), "Scene: chair ahead; path clear.")

    def test_completed_utterance_uses_matching_vision_report(self):
        agent = FakeAgent()
        app = RobotApp(agent, FakeAudio())
        vision: Future[str] = Future()
        vision.set_result("Scene: table left; center path clear.")
        app.speech_generation = 1
        app.turn_generation = 1
        app.vision_futures[1] = vision
        app._process_turn(b"test wav", 1)
        self.assertEqual(
            agent.turns,
            [("test phrase", "Scene: table left; center path clear.")],
        )

    def test_action_executes_only_after_announcement_finishes(self):
        app = RobotApp(FakeAgent(), FakeAudio())
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("move", 30, 80))
        lines = output.getvalue().splitlines()
        proposed = next(i for i, line in enumerate(lines) if line.startswith("[PROPOSED]"))
        acting = next(i for i, line in enumerate(lines) if line == "[STATE] acting")
        executed = next(i for i, line in enumerate(lines) if line.startswith("[TOOL]"))
        self.assertLess(proposed, acting)
        self.assertLess(acting, executed)
        self.assertIsNotNone(result.content)
        self.assertEqual(app.state, State.PROCESSING)

    def test_microphone_is_not_used_during_announcement(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("turn", 50, -90))
        self.assertEqual(audio.played, [b"fake pcm"])
        self.assertIsNone(result.interruption)
        self.assertIn("[TOOL]", output.getvalue())
        self.assertEqual(app.state, State.PROCESSING)

    def test_new_speech_cancels_pending_action(self):
        audio = FakeAudio()
        app = RobotApp(FakeAgent(), audio)
        app.worker_busy.set()
        app._on_speech_start()
        output = io.StringIO()
        with redirect_stdout(output):
            result = app._handle_action(MotionCall("move", 30, 80))
        self.assertEqual(result.interruption, b"")
        self.assertEqual(audio.played, [])
        self.assertIn("[CANCELLED]", output.getvalue())
        self.assertNotIn("[TOOL]", output.getvalue())

    def test_speech_during_tts_request_cancels_before_playback(self):
        audio = FakeAudio()
        agent = FakeAgent()
        app = RobotApp(agent, audio)

        def interrupted_synthesis(text):
            app.worker_busy.set()
            app._on_speech_start()
            return io.BytesIO(b"fake pcm")

        agent.synthesize = interrupted_synthesis
        result = app._handle_action(MotionCall("turn", 40, 90))
        self.assertEqual(result.interruption, b"")
        self.assertEqual(audio.played, [])

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
