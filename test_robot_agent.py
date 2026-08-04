import io
import json
import math
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from robot_agent import (
    ActionResult,
    Camera,
    GroqRobot,
    MotionCall,
    announcement,
    execute_motion,
    validate_motion,
)


def tool_call(call_id, name, arguments):
    return SimpleNamespace(
        id=call_id,
        type="function",
        function=SimpleNamespace(name=name, arguments=json.dumps(arguments)),
    )


def response(*, content=None, tool_calls=None):
    message = SimpleNamespace(content=content, tool_calls=tool_calls)
    return SimpleNamespace(choices=[SimpleNamespace(message=message)])


class FakeCompletions:
    def __init__(self, responses):
        self.responses = list(responses)
        self.calls = []

    def create(self, **kwargs):
        self.calls.append(kwargs)
        return self.responses.pop(0)


class FakeClient:
    def __init__(self, responses):
        self.completions = FakeCompletions(responses)
        self.chat = SimpleNamespace(completions=self.completions)


class FakeCamera:
    def __init__(self, value="data:image/jpeg;base64,abc", error=None):
        self.value = value
        self.error = error
        self.closed = False

    def data_url(self):
        if self.error:
            raise self.error
        return self.value

    def close(self):
        self.closed = True

    def set_index(self, index):
        self.index = index
        self.closed = True


class MotionValidationTests(unittest.TestCase):
    def test_signed_move_and_turn(self):
        move = validate_motion("move", {"speed": 30, "distance": -80})
        turn = validate_motion("turn", {"speed": 45, "angle": 90})
        self.assertEqual(move, MotionCall("move", 30, -80.0))
        self.assertEqual(turn, MotionCall("turn", 45, 90.0))
        self.assertIn("backward 80 centimeters", announcement(move))
        self.assertIn("left 90 degrees", announcement(turn))

    def test_invalid_motion_values(self):
        invalid = [
            ("move", {"speed": 0, "distance": 1}),
            ("move", {"speed": True, "distance": 1}),
            ("move", {"speed": 10, "distance": 0}),
            ("move", {"speed": 10, "distance": 501}),
            ("turn", {"speed": 10, "angle": -361}),
            ("turn", {"speed": 10, "angle": math.inf}),
            ("dance", {"speed": 10, "angle": 1}),
            ("turn", {"speed": 10, "angle": 1, "extra": 2}),
        ]
        for name, arguments in invalid:
            with self.subTest(name=name, arguments=arguments):
                with self.assertRaises(ValueError):
                    validate_motion(name, arguments)

    def test_motion_honors_speech_stop_event(self):
        stop = threading.Event()
        stop.set()
        result = json.loads(execute_motion(MotionCall("move", 30, 80), stop))
        self.assertEqual(result["status"], "cancelled")


class AgentLoopTests(unittest.TestCase):
    def make_agent(self, responses, camera=None):
        client = FakeClient(responses)
        agent = GroqRobot("unused", client=client, camera=camera or FakeCamera())
        return agent, client

    def test_qwen_describes_image_then_gpt_oss_receives_text_only(self):
        agent, client = self.make_agent(
            [
                response(content="Scene: desk centered; path clear."),
                response(content="I can see a desk."),
            ],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        with patch("robot_agent.Path.write_bytes") as write_bytes:
            scene = agent.describe_scene()
        outcome = agent.run_turn("What can you see?", scene, lambda _: None)
        self.assertEqual(outcome.reply, "I can see a desk.")
        write_bytes.assert_called_once_with(b"jpeg")
        vision_call, agent_call = client.completions.calls
        self.assertEqual(vision_call["model"], "qwen/qwen3.6-27b")
        self.assertEqual(vision_call["messages"][-1]["content"][1]["type"], "image_url")
        self.assertEqual(agent_call["model"], "openai/gpt-oss-120b")
        self.assertIsInstance(agent_call["messages"][-1]["content"], str)
        self.assertIn(scene, agent_call["messages"][-1]["content"])
        self.assertEqual(agent_call["reasoning_effort"], "low")

    def test_deepgram_tts_uses_one_streaming_pcm_request(self):
        agent, _ = self.make_agent([])
        agent.deepgram_api_key = "deepgram-test"
        pcm = b"\x01\x00\x02\x00"
        response_stream = io.BytesIO(pcm)
        with patch("robot_agent.urlopen", return_value=response_stream) as open_url:
            stream = agent.synthesize("First sentence. Second sentence!")
        request = open_url.call_args.args[0]
        self.assertIs(stream, response_stream)
        self.assertEqual(json.loads(request.data), {"text": "First sentence. Second sentence!"})
        self.assertEqual(request.get_header("Authorization"), "Token deepgram-test")
        self.assertIn("model=aura-2-thalia-en", request.full_url)
        self.assertIn("container=none", request.full_url)
        self.assertEqual(stream.read(), pcm)

    def test_vision_report_is_capped_at_one_hundred_words(self):
        agent, _ = self.make_agent(
            [response(content=" ".join(f"word{i}" for i in range(130)))],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        with patch("robot_agent.Path.write_bytes"):
            report = agent.describe_scene()
        self.assertEqual(len(report.removesuffix("…").split()), 100)

    def test_tool_executes_before_final_response(self):
        call = tool_call("one", "move", {"speed": 25, "distance": 40})
        agent, client = self.make_agent(
            [response(tool_calls=[call]), response(content="Movement simulated.")]
        )
        seen = []

        def act(motion):
            seen.append(motion)
            return ActionResult(content=json.dumps({"status": "simulated"}))

        outcome = agent.run_turn("Move a little", "Path clear.", act)
        self.assertEqual(seen, [MotionCall("move", 25, 40.0)])
        self.assertEqual(outcome.reply, "Movement simulated.")
        second_messages = client.completions.calls[1]["messages"]
        self.assertTrue(any(message["role"] == "tool" for message in second_messages))

    def test_malformed_tool_never_reaches_action(self):
        bad = SimpleNamespace(
            id="bad",
            type="function",
            function=SimpleNamespace(name="move", arguments="not json"),
        )
        agent, _ = self.make_agent(
            [response(tool_calls=[bad]), response(content="Please try that again.")]
        )
        outcome = agent.run_turn(
            "Move", "Path clear.", lambda _: self.fail("invalid call reached executor")
        )
        self.assertEqual(outcome.reply, "Please try that again.")

    def test_four_call_limit(self):
        calls = [tool_call(str(i), "turn", {"speed": 20, "angle": 10}) for i in range(5)]
        agent, client = self.make_agent(
            [response(tool_calls=calls), response(content="Four turns simulated.")]
        )
        executed = []
        outcome = agent.run_turn(
            "Turn repeatedly",
            "Path clear.",
            lambda call: executed.append(call) or ActionResult(content="{}"),
        )
        self.assertEqual(len(executed), 4)
        self.assertEqual(outcome.reply, "Four turns simulated.")
        self.assertNotIn("tools", client.completions.calls[1])

    def test_interruption_cancels_turn_without_second_request(self):
        call = tool_call("one", "move", {"speed": 25, "distance": 40})
        agent, client = self.make_agent([response(tool_calls=[call])])
        outcome = agent.run_turn(
            "Move",
            "Path clear.",
            lambda _: ActionResult(interruption=b"new utterance"),
        )
        self.assertEqual(outcome.interruption, b"new utterance")
        self.assertEqual(len(client.completions.calls), 1)
        self.assertIn("cancelled", agent.history[-1]["content"])

    def test_camera_failure_falls_back(self):
        agent, _ = self.make_agent([], FakeCamera(error=RuntimeError("no camera")))
        self.assertEqual(agent.describe_scene(), "Visual context unavailable.")
        self.assertEqual(agent.describe_scene(), "Visual context unavailable.")


if __name__ == "__main__":
    unittest.main()
