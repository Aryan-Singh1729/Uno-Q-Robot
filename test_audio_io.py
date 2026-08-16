import unittest
from collections import deque

import numpy as np

from audio_io import (
    END_SILENCE_FRAMES,
    UtteranceDetector,
    VoiceIO,
    amplify_pcm,
    maximize_pcm,
    pcm_rms,
    pcm_to_wav,
    resample_pcm,
)


class UtteranceDetectorTests(unittest.TestCase):
    def test_isolated_noise_does_not_start(self):
        detector = UtteranceDetector()
        for index in range(80):
            started, audio = detector.push(b"x", index in {5, 20, 35, 50, 65})
            self.assertFalse(started)
            self.assertIsNone(audio)

    def test_preroll_and_endpoint_silence_are_retained(self):
        detector = UtteranceDetector()
        started_count = 0
        completed = None
        for _ in range(14):
            detector.push(b"p", False)
        for _ in range(10):
            started, audio = detector.push(b"v", True)
            started_count += started
            completed = audio if audio is not None else completed
        for _ in range(END_SILENCE_FRAMES):
            _, audio = detector.push(b"s", False)
            completed = audio if audio is not None else completed
        self.assertEqual(started_count, 1)
        self.assertIsNotNone(completed)
        self.assertTrue(completed.startswith(b"p"))
        self.assertTrue(completed.endswith(b"s" * END_SILENCE_FRAMES))

    def test_pause_shorter_than_threshold_does_not_finish(self):
        detector = UtteranceDetector()
        for _ in range(10):
            detector.push(b"v", True)
        for _ in range(END_SILENCE_FRAMES - 1):
            _, audio = detector.push(b"s", False)
            self.assertIsNone(audio)
        for _ in range(3):
            detector.push(b"v", True)
        completed = None
        for _ in range(END_SILENCE_FRAMES):
            _, audio = detector.push(b"s", False)
            if audio is not None:
                completed = audio
        self.assertTrue(completed)

    def test_isolated_vad_spikes_do_not_restart_endpoint(self):
        detector = UtteranceDetector()
        for _ in range(10):
            detector.push(b"v", True)
        completed = None
        for index in range(END_SILENCE_FRAMES):
            _, audio = detector.push(b"n", index in {10, 20, 30})
            if audio is not None:
                completed = audio
        self.assertTrue(completed)

    def test_short_false_trigger_is_discarded(self):
        detector = UtteranceDetector()
        for _ in range(6):
            detector.push(b"v", True)
        completed = None
        # Four silent frames complete the 10-frame start window, followed by
        # the configured endpoint silence.
        for _ in range(4 + END_SILENCE_FRAMES):
            _, audio = detector.push(b"s", False)
            if audio is not None:
                completed = audio
        self.assertEqual(completed, b"")

    def test_maximum_duration_finishes_recording(self):
        detector = UtteranceDetector(max_utterance_frames=25)
        completed = None
        for _ in range(30):
            _, audio = detector.push(b"v", True)
            if audio is not None:
                completed = audio
                break
        self.assertIsNotNone(completed)
        self.assertEqual(len(completed), 25)

    def test_pcm_wav_header(self):
        wav = pcm_to_wav(b"\x00\x00" * 480)
        self.assertEqual(wav[:4], b"RIFF")
        self.assertIn(b"WAVE", wav[:16])

    def test_energy_gate_rejects_vad_positive_quiet_frame(self):
        voice = VoiceIO.__new__(VoiceIO)
        voice.speech_rms_threshold = 1_000
        voice.vad = type("Vad", (), {"is_speech": lambda self, frame, rate: True})()
        quiet = b"\x64\x00" * 480
        loud = b"\xd0\x07" * 480
        self.assertAlmostEqual(pcm_rms(quiet), 100)
        self.assertFalse(voice._is_speech(quiet))
        self.assertTrue(voice._is_speech(loud))

    def test_loud_calibration_cannot_make_speech_detection_impossible(self):
        voice = VoiceIO.__new__(VoiceIO)
        voice._fixed_capture = lambda seconds: [b"\xff\x7f" * 480] * 10
        self.assertEqual(voice.calibrate_input(), 3_000)

    def test_pcm_resamples_for_fixed_rate_output(self):
        source = np.arange(240, dtype=np.int16).tobytes()
        converted = resample_pcm(
            source,
            channels=1,
            dtype="int16",
            source_rate=24_000,
            target_rate=48_000,
        )
        self.assertEqual(len(converted), len(source) * 2)

    def test_pcm_gain_is_applied_and_clipped(self):
        source = np.array([1000, -1000, 30000, -30000], dtype=np.int16).tobytes()
        result = np.frombuffer(amplify_pcm(source, 1.4), dtype=np.int16).tolist()
        self.assertEqual(result, [1400, -1400, 32767, -32768])

    def test_pcm_can_be_peak_normalized_without_clipping(self):
        source = np.array([1000, -2000, 500], dtype=np.int16).tobytes()
        result = np.frombuffer(maximize_pcm(source), dtype=np.int16)
        self.assertEqual(int(np.max(np.abs(result))), 32000)

    def test_usb_camera_microphone_is_preferred_over_analog_default(self):
        class Default:
            device = (0, 2)

        class SoundDevice:
            default = Default()

            @staticmethod
            def query_devices():
                return [
                    {"name": "Qualcomm analog input", "max_input_channels": 2},
                    {"name": "USB Camera microphone", "max_input_channels": 1},
                    {"name": "HDMI output", "max_input_channels": 0},
                ]

        voice = VoiceIO.__new__(VoiceIO)
        voice.sd = SoundDevice
        self.assertEqual(voice._automatic_input_device(), 1)

    def test_emeet_smartcam_mic_beats_generic_usb_audio(self):
        class Default:
            device = (1, 1)

        class SoundDevice:
            default = Default()

            @staticmethod
            def query_devices():
                return [
                    {"name": "EMEET SmartCam C950: USB Audio", "max_input_channels": 1},
                    {"name": "USB PnP Sound Device: Audio", "max_input_channels": 1},
                ]

        voice = VoiceIO.__new__(VoiceIO)
        voice.sd = SoundDevice
        self.assertEqual(voice._automatic_input_device(), 0)

    def test_webcam_capture_is_resampled_to_vad_rate(self):
        voice = VoiceIO.__new__(VoiceIO)
        voice.input_rate = 48_000
        source = np.arange(1_440, dtype=np.int16).tobytes()
        converted = voice._capture_callback(source)
        self.assertEqual(len(converted), 480 * 2)

    def test_usb_input_recovery_refreshes_portaudio_and_reselects_device(self):
        calls = []

        class SoundDevice:
            class PortAudioError(Exception):
                pass

            @staticmethod
            def stop():
                calls.append("stop")

            @staticmethod
            def _terminate():
                calls.append("terminate")

            @staticmethod
            def _initialize():
                calls.append("initialize")

        voice = VoiceIO.__new__(VoiceIO)
        voice.sd = SoundDevice
        voice.requested_input_device = None
        voice.input_rate = 16_000
        voice.set_input_device = lambda value: calls.append(("select", value)) or "EMEET"
        self.assertEqual(voice.recover_input_device(), "EMEET")
        self.assertEqual(calls, ["stop", "terminate", "initialize", ("select", None)])
        self.assertTrue(voice.is_input_error(SoundDevice.PortAudioError("disconnected")))

    def test_pcm_response_is_played_as_chunks_arrive(self):
        writes = []

        class Output:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def write(self, pcm):
                writes.append(pcm)

        class SoundDevice:
            class PortAudioError(Exception):
                pass

            @staticmethod
            def check_output_settings(**_kwargs):
                pass

            @staticmethod
            def RawOutputStream(**_kwargs):
                return Output()

        class Stream:
            chunks = deque([b"\x01\x00", b"\x02\x00"])

            def read1(self, _size):
                return self.chunks.popleft() if self.chunks else b""

            read = read1

        voice = VoiceIO.__new__(VoiceIO)
        voice.sd = SoundDevice
        voice.output_device = None
        voice.playback_gain = 1.4
        voice.play(Stream())
        self.assertEqual(len(writes), 2)
        self.assertEqual(
            [np.frombuffer(item, dtype=np.int16).tolist() for item in writes],
            [[1], [3]],
        )

    def test_incomplete_pcm_response_is_rejected_before_playback(self):
        class Output:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return None

            def write(self, _pcm):
                return None

        class SoundDevice:
            class PortAudioError(Exception):
                pass

            @staticmethod
            def check_output_settings(**_kwargs):
                pass

            @staticmethod
            def RawOutputStream(**_kwargs):
                return Output()

        class Stream:
            def read(self, _size):
                value, self.value = getattr(self, "value", b"\x01"), b""
                return value

        voice = VoiceIO.__new__(VoiceIO)
        voice.sd = SoundDevice
        voice.output_device = None
        voice.playback_gain = 1.0
        with self.assertRaisesRegex(RuntimeError, "incomplete PCM"):
            voice.play(Stream())


if __name__ == "__main__":
    unittest.main()
