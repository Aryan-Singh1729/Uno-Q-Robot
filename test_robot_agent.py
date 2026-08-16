import io
import json
import math
import sys
import threading
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch

from robot_agent import (
    ActionResult,
    Camera,
    GroqRobot,
    MotionCall,
    TargetMission,
    announcement,
    execute_motion,
    parse_api_keys,
    parse_direct_motion,
    parse_target_mission,
    validate_inspection,
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
        result = self.responses.pop(0)
        if isinstance(result, BaseException):
            raise result
        return result


class FakeClient:
    def __init__(self, responses):
        self.completions = FakeCompletions(responses)
        self.chat = SimpleNamespace(completions=self.completions)


class FakeCamera:
    def __init__(self, value="data:image/jpeg;base64,abc", error=None):
        self.value = value
        self.error = error
        self.closed = False
        self.calls = 0

    def data_url(self):
        self.calls += 1
        if self.error:
            raise self.error
        return self.value

    def close(self):
        self.closed = True

    def set_index(self, index):
        self.index = index
        self.closed = True


class FakeRateLimitError(Exception):
    pass


class MotionValidationTests(unittest.TestCase):
    def test_camera_falls_back_when_first_video_node_times_out(self):
        opened = []

        class Capture:
            def __init__(self, index):
                self.index = index
                self.released = False

            def set(self, *_args):
                return True

            def isOpened(self):
                return True

            def read(self):
                return (self.index == 1, object())

            def release(self):
                self.released = True

        class Encoded:
            @staticmethod
            def tobytes():
                return b"jpeg"

        fake_cv2 = SimpleNamespace(
            CAP_V4L2=200,
            CAP_PROP_FRAME_WIDTH=3,
            CAP_PROP_FRAME_HEIGHT=4,
            CAP_PROP_BUFFERSIZE=38,
            CAP_PROP_FOURCC=6,
            IMWRITE_JPEG_QUALITY=1,
            VideoCapture=lambda index, _backend=None: opened.append(Capture(index)) or opened[-1],
            VideoWriter_fourcc=lambda *_args: 123,
            imencode=lambda *_args: (True, Encoded()),
        )
        camera = Camera(0)
        with patch.dict(sys.modules, {"cv2": fake_cv2}), patch.object(
            Camera, "_device_indices", return_value=[0, 1]
        ):
            result = camera.data_url()
        self.assertEqual(camera.index, 1)
        self.assertEqual(result, "data:image/jpeg;base64,anBlZw==")
        self.assertTrue(opened[0].released)

    def test_comma_separated_api_keys(self):
        self.assertEqual(parse_api_keys(" first, second ,, third "), ["first", "second", "third"])
        with self.assertRaises(ValueError):
            parse_api_keys(" , ")

    def test_signed_move_and_turn(self):
        move = validate_motion("move", {"speed": 30, "duration_seconds": -8})
        turn = validate_motion("turn", {"duration_seconds": 0.5})
        self.assertEqual(move, MotionCall("move", 30, -8.0))
        self.assertEqual(turn, MotionCall("turn", 50, 0.5))
        self.assertIn("backward for 8 seconds", announcement(move))
        self.assertIn("left for 0.5 seconds", announcement(turn))

    def test_move_requires_speed_and_turn_uses_fixed_fifty_percent(self):
        with self.assertRaises(ValueError):
            validate_motion("move", {"duration_seconds": 5})
        self.assertEqual(
            validate_motion("turn", {"duration_seconds": -2}),
            MotionCall("turn", 50, -2.0),
        )

    def test_invalid_motion_values(self):
        invalid = [
            ("move", {"speed": 0, "duration_seconds": 1}),
            ("move", {"speed": True, "duration_seconds": 1}),
            ("move", {"speed": 10, "duration_seconds": 0}),
            ("move", {"speed": 10, "duration_seconds": 61}),
            ("turn", {"duration_seconds": -61}),
            ("spin", {"speed": 10, "seconds": 121}),
            ("move", {"speed": 101, "duration_seconds": 1}),
            ("turn", {"duration_seconds": math.inf}),
            ("dance", {"duration_seconds": 1}),
            ("turn", {"speed": 10, "duration_seconds": 1}),
            ("turn", {"duration_seconds": 1, "extra": 2}),
        ]
        for name, arguments in invalid:
            with self.subTest(name=name, arguments=arguments):
                with self.assertRaises(ValueError):
                    validate_motion(name, arguments)

    def test_motion_honors_speech_stop_event(self):
        stop = threading.Event()
        stop.set()
        result = json.loads(execute_motion(MotionCall("move", 30, 8), stop))
        self.assertEqual(result["status"], "cancelled")

    def test_inspection_question_validation(self):
        self.assertEqual(validate_inspection({"question": "  Where is the chair?  "}), "Where is the chair?")
        for arguments in ({}, {"question": ""}, {"question": 3}, {"question": "x" * 301}):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                validate_inspection(arguments)

    def test_direct_motion_parser_keeps_directions_deterministic(self):
        self.assertEqual(
            parse_direct_motion("move forward for 5 seconds"),
            MotionCall("move", 20, 5.0),
        )
        self.assertEqual(
            parse_direct_motion("move backwards for 2 seconds at 35 percent"),
            MotionCall("move", 35, -2.0),
        )
        self.assertEqual(
            parse_direct_motion("move forward for five seconds at twenty percent"),
            MotionCall("move", 20, 5.0),
        )
        self.assertEqual(
            parse_direct_motion("rotate right for 10 seconds at 40%"),
            MotionCall("turn", 50, -10.0),
        )
        self.assertIsNone(parse_direct_motion("what do you see in front of you?"))
        with self.assertRaisesRegex(ValueError, "distance commands are disabled"):
            parse_direct_motion("move forward by 50 centimeters")
        with self.assertRaisesRegex(ValueError, "turn duration"):
            parse_direct_motion("turn left")

    def test_target_mission_parser(self):
        self.assertEqual(
            parse_target_mission("Find a shoe in this room and move towards it."),
            TargetMission("shoe", "toward"),
        )
        self.assertEqual(
            parse_target_mission("locate the sofa and then move away from it"),
            TargetMission("sofa", "away"),
        )


class AgentLoopTests(unittest.TestCase):
    def make_agent(self, responses, camera=None):
        client = FakeClient(responses)
        agent = GroqRobot("unused", client=client, camera=camera or FakeCamera())
        return agent, client

    def test_physical_mode_prompt_forbids_camera_motion_estimates(self):
        agent, _ = self.make_agent([])
        agent.physical_motion = True
        self.assertIn("You are WALL-E", agent.system_prompt)
        self.assertIn("control real motors", agent.system_prompt)
        self.assertIn("Never claim exact physical distance or angle", agent.system_prompt)
        self.assertNotIn("This is a terminal simulator", agent.system_prompt)

    def test_gpt_oss_requests_targeted_qwen_inspection(self):
        inspect = tool_call("vision", "inspect_scene", {"question": "Where is the desk?"})
        agent, client = self.make_agent(
            [
                response(tool_calls=[inspect]),
                response(content="Scene: desk centered; path clear."),
                response(content="I can see a desk."),
            ],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        output = io.StringIO()
        with patch("robot_agent.Path.write_bytes") as write_bytes, redirect_stdout(output):
            outcome = agent.run_turn("What can you see?", lambda _: None)
        self.assertEqual(outcome.reply, "I can see a desk.")
        write_bytes.assert_called_once_with(b"jpeg")
        agent_call, vision_call, final_call = client.completions.calls
        self.assertEqual(agent_call["model"], "openai/gpt-oss-120b")
        self.assertNotIn("Camera report", agent_call["messages"][-1]["content"])
        self.assertFalse(agent_call["parallel_tool_calls"])
        self.assertEqual(vision_call["model"], "qwen/qwen3.6-27b")
        self.assertEqual(vision_call["messages"][-1]["content"][1]["type"], "image_url")
        self.assertIn("Where is the desk?", vision_call["messages"][-1]["content"][0]["text"])
        self.assertIn("[VISION] prompt:\nAnswer this visual question", output.getvalue())
        self.assertIn("Where is the desk?", output.getvalue())
        tool_result = next(message for message in final_call["messages"] if message["role"] == "tool")
        self.assertEqual(json.loads(tool_result["content"])["observation"], "Scene: desk centered; path clear.")
        self.assertEqual(agent_call["reasoning_effort"], "low")

    def test_conversation_does_not_capture_or_call_qwen(self):
        camera = FakeCamera(value="data:image/jpeg;base64,anBlZw==")
        agent, client = self.make_agent([response(content="Hello!")], camera)
        outcome = agent.run_turn("Hello", lambda _: None)
        self.assertEqual(outcome.reply, "Hello!")
        self.assertEqual(camera.calls, 0)
        self.assertEqual(len(client.completions.calls), 1)

    def test_qwen_llm_uses_supported_reasoning_effort(self):
        agent, client = self.make_agent([response(content="Hello!")])
        agent.llm_model = "qwen/qwen3.6-27b"
        self.assertEqual(agent.run_turn("Hello", lambda _: None).reply, "Hello!")
        self.assertEqual(client.completions.calls[0]["reasoning_effort"], "default")

    def test_cerebras_glm_disables_reasoning_with_supported_value(self):
        agent, client = self.make_agent([response(content="Hello!")])
        agent.llm_model = "zai-glm-4.7"
        self.assertEqual(agent.run_turn("Hello", lambda _: None).reply, "Hello!")
        self.assertEqual(client.completions.calls[0]["reasoning_effort"], "none")

    def test_target_locator_parses_strict_geometry_without_safety_advice(self):
        agent, client = self.make_agent(
            [
                response(
                    content=(
                        '{"visible":true,"position":"left","scale":"medium",'
                        '"reached":false,"description":"black shoe on floor"}'
                    )
                )
            ],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        observation = agent.locate_target("shoe")
        self.assertTrue(observation.visible)
        self.assertEqual(observation.position, "left")
        self.assertEqual(observation.scale, "medium")
        prompt = client.completions.calls[0]["messages"][-1]["content"][0]["text"]
        self.assertIn("Do not evaluate obstacles", prompt)

    def test_rate_limited_keys_rotate_and_wrap(self):
        first = FakeClient([FakeRateLimitError(), response(content="Back on key one.")])
        second = FakeClient([FakeRateLimitError()])
        third = FakeClient([response(content="Key three works."), FakeRateLimitError()])
        agent = GroqRobot("unused", client=first, camera=FakeCamera())
        agent._clients = [first, second, third]
        agent._rate_limit_error = FakeRateLimitError

        self.assertEqual(agent.run_turn("First", lambda _: None).reply, "Key three works.")
        self.assertEqual(agent._client_index, 2)
        self.assertEqual(agent.run_turn("Second", lambda _: None).reply, "Back on key one.")
        self.assertEqual(agent._client_index, 0)

    def test_cerebras_llm_keys_rotate_and_wrap(self):
        groq = FakeClient([])
        first = FakeClient([FakeRateLimitError(), response(content="Back on key one.")])
        second = FakeClient([response(content="Key two works."), FakeRateLimitError()])
        agent = GroqRobot(
            "unused",
            client=groq,
            cerebras_client=first,
            camera=FakeCamera(),
        )
        agent._cerebras_clients = [first, second]
        agent._cerebras_rate_limit_error = FakeRateLimitError

        self.assertEqual(agent.llm_model, "gpt-oss-120b")
        self.assertEqual(agent.run_turn("First", lambda _: None).reply, "Key two works.")
        self.assertEqual(agent._cerebras_client_index, 1)
        self.assertEqual(agent.run_turn("Second", lambda _: None).reply, "Back on key one.")
        self.assertEqual(agent._cerebras_client_index, 0)
        self.assertEqual(groq.completions.calls, [])

    def test_fully_rate_limited_pool_stops_after_one_cycle(self):
        clients = [FakeClient([FakeRateLimitError()]) for _ in range(3)]
        agent = GroqRobot("unused", client=clients[0], camera=FakeCamera())
        agent._clients = clients
        agent._rate_limit_error = FakeRateLimitError

        with self.assertRaises(FakeRateLimitError):
            agent.run_turn("Hello", lambda _: None)
        self.assertEqual([len(client.completions.calls) for client in clients], [1, 1, 1])
        self.assertEqual(agent._client_index, 0)

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
            result = json.loads(agent.inspect_scene("Describe the relevant objects."))
        self.assertEqual(len(result["observation"].removesuffix("…").split()), 100)

    def test_tool_executes_before_final_response(self):
        call = tool_call("one", "move", {"speed": 25, "duration_seconds": 4})
        agent, client = self.make_agent(
            [response(tool_calls=[call]), response(content="Movement simulated.")]
        )
        seen = []

        def act(motion):
            seen.append(motion)
            return ActionResult(content=json.dumps({"status": "simulated"}))

        outcome = agent.run_turn("Move a little", act)
        self.assertEqual(seen, [MotionCall("move", 25, 4.0)])
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
            "Move", lambda _: self.fail("invalid call reached executor")
        )
        self.assertEqual(outcome.reply, "Please try that again.")

    def test_fresh_inspection_is_required_between_motions(self):
        calls = [tool_call(str(i), "turn", {"duration_seconds": 0.5}) for i in range(2)]
        agent, client = self.make_agent(
            [*(response(tool_calls=[call]) for call in calls), response(content="Inspection required.")]
        )
        executed = []
        outcome = agent.run_turn(
            "Turn repeatedly",
            lambda call: executed.append(call) or ActionResult(content="{}"),
        )
        self.assertEqual(len(executed), 1)
        self.assertEqual(outcome.reply, "Inspection required.")
        tool_result = client.completions.calls[-1]["messages"][-1]["content"]
        self.assertIn("fresh camera inspection required", tool_result)

    def test_interruption_cancels_turn_without_second_request(self):
        call = tool_call("one", "move", {"speed": 25, "duration_seconds": 4})
        agent, client = self.make_agent([response(tool_calls=[call])])
        outcome = agent.run_turn(
            "Move",
            lambda _: ActionResult(interruption=b"new utterance"),
        )
        self.assertEqual(outcome.interruption, b"new utterance")
        self.assertEqual(len(client.completions.calls), 1)
        self.assertIn("cancelled", agent.history[-1]["content"])

    def test_multiple_inspections_are_unlimited_and_use_fresh_frames(self):
        first = tool_call("one", "inspect_scene", {"question": "What is ahead?"})
        second = tool_call("two", "inspect_scene", {"question": "Is the path clear?"})
        camera = FakeCamera(value="data:image/jpeg;base64,anBlZw==")
        agent, client = self.make_agent(
            [
                response(tool_calls=[first]),
                response(content="A chair is ahead."),
                response(tool_calls=[second]),
                response(content="The path is clear."),
                response(content="The chair is ahead and the path looks clear."),
            ],
            camera,
        )
        with patch("robot_agent.Path.write_bytes"):
            outcome = agent.run_turn("Can I approach the chair?", lambda _: None)
        self.assertEqual(camera.calls, 2)
        self.assertEqual(outcome.reply, "The chair is ahead and the path looks clear.")
        self.assertEqual(
            [call["model"] for call in client.completions.calls],
            [
                "openai/gpt-oss-120b",
                "qwen/qwen3.6-27b",
                "openai/gpt-oss-120b",
                "qwen/qwen3.6-27b",
                "openai/gpt-oss-120b",
            ],
        )

    def test_inspection_does_not_consume_motion_limit(self):
        inspect1 = tool_call("vision1", "inspect_scene", {"question": "Is the path clear?"})
        inspect2 = tool_call("vision2", "inspect_scene", {"question": "Is it still clear?"})
        motions = [
            tool_call(str(i), "move", {"speed": 20, "duration_seconds": 1})
            for i in range(2)
        ]
        agent, client = self.make_agent(
            [
                response(tool_calls=[inspect1]),
                response(content="The path is clear."),
                response(tool_calls=[motions[0]]),
                response(tool_calls=[inspect2]),
                response(content="The path remains clear."),
                response(tool_calls=[motions[1]]),
                response(content="Two movements simulated."),
            ],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        executed = []
        with patch("robot_agent.Path.write_bytes"):
            outcome = agent.run_turn(
                "Move four times if the path is clear",
                lambda call: executed.append(call) or ActionResult(content="{}"),
            )
        self.assertEqual(len(executed), 2)
        self.assertEqual(outcome.reply, "Two movements simulated.")
        first_post_vision_request = client.completions.calls[2]
        observation = next(
            message for message in first_post_vision_request["messages"] if message["role"] == "tool"
        )
        self.assertEqual(json.loads(observation["content"])["observation"], "The path is clear.")

    def test_malformed_inspection_never_uses_camera(self):
        bad = tool_call("bad", "inspect_scene", {"question": ""})
        camera = FakeCamera(value="data:image/jpeg;base64,anBlZw==")
        agent, _ = self.make_agent(
            [response(tool_calls=[bad]), response(content="Please clarify.")],
            camera,
        )
        outcome = agent.run_turn("Look", lambda _: None)
        self.assertEqual(outcome.reply, "Please clarify.")
        self.assertEqual(camera.calls, 0)

    def test_qwen_failure_is_returned_to_gpt_oss(self):
        inspect = tool_call("vision", "inspect_scene", {"question": "What is ahead?"})
        agent, client = self.make_agent(
            [
                response(tool_calls=[inspect]),
                response(content=""),
                response(content="I couldn't inspect the scene."),
            ],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        with patch("robot_agent.Path.write_bytes"):
            outcome = agent.run_turn("What is ahead?", lambda _: None)
        self.assertEqual(outcome.reply, "I couldn't inspect the scene.")
        tool_result = next(
            message
            for message in client.completions.calls[-1]["messages"]
            if message["role"] == "tool"
        )
        self.assertEqual(json.loads(tool_result["content"])["error"], "vision_failed")

    def test_speech_during_qwen_discards_result(self):
        inspect = tool_call("vision", "inspect_scene", {"question": "What is ahead?"})
        agent, client = self.make_agent(
            [response(tool_calls=[inspect]), response(content="A chair is ahead.")],
            FakeCamera(value="data:image/jpeg;base64,anBlZw=="),
        )
        checks = 0

        def cancelled():
            nonlocal checks
            checks += 1
            return checks >= 5

        with patch("robot_agent.Path.write_bytes"):
            outcome = agent.run_turn("What is ahead?", lambda _: None, cancelled)
        self.assertEqual(outcome.interruption, b"")
        self.assertEqual(len(client.completions.calls), 2)

    def test_parallel_tool_batch_executes_nothing(self):
        inspect = tool_call("vision", "inspect_scene", {"question": "Is the path clear?"})
        move = tool_call("move", "move", {"speed": 20, "duration_seconds": 3})
        camera = FakeCamera(value="data:image/jpeg;base64,anBlZw==")
        agent, _ = self.make_agent(
            [response(tool_calls=[inspect, move]), response(content="I'll wait.")],
            camera,
        )
        outcome = agent.run_turn("Move if clear", lambda _: self.fail("motion executed"))
        self.assertEqual(outcome.reply, "I'll wait.")
        self.assertEqual(camera.calls, 0)

    def test_camera_failure_falls_back(self):
        agent, _ = self.make_agent([], FakeCamera(error=RuntimeError("no camera")))
        result = json.loads(agent.inspect_scene("What is ahead?"))
        self.assertEqual(result["error"], "camera_unavailable")


if __name__ == "__main__":
    unittest.main()
