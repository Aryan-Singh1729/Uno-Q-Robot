import unittest
from collections import deque

import numpy as np

from audio_io import UtteranceDetector, VoiceIO, pcm_rms, pcm_to_wav, resample_pcm


class UtteranceDetectorTests(unittest.TestCase):
    def test_isolated_noise_does_not_start(self):
        detector = UtteranceDetector()
        for index in range(80):
            started, audio = detector.push(b"x", index in {5, 20, 35, 50, 65})
            self.assertFalse(started)
            self.assertIsNone(audio)

    def test_preroll_and_three_second_silence_are_retained(self):
        detector = UtteranceDetector()
        started_count = 0
        completed = None
        for _ in range(14):
            detector.push(b"p", False)
        for _ in range(10):
            started, audio = detector.push(b"v", True)
            started_count += started
            completed = audio if audio is not None else completed
        for _ in range(100):
            _, audio = detector.push(b"s", False)
            completed = audio if audio is not None else completed
        self.assertEqual(started_count, 1)
        self.assertIsNotNone(completed)
        self.assertTrue(completed.startswith(b"p"))
        self.assertTrue(completed.endswith(b"s" * 100))

    def test_pause_shorter_than_threshold_does_not_finish(self):
        detector = UtteranceDetector()
        for _ in range(10):
            detector.push(b"v", True)
        for _ in range(99):
            _, audio = detector.push(b"s", False)
            self.assertIsNone(audio)
        for _ in range(3):
            detector.push(b"v", True)
        completed = None
        for _ in range(100):
            _, audio = detector.push(b"s", False)
            if audio is not None:
                completed = audio
        self.assertTrue(completed)

    def test_isolated_vad_spikes_do_not_restart_endpoint(self):
        detector = UtteranceDetector()
        for _ in range(10):
            detector.push(b"v", True)
        completed = None
        for index in range(100):
            _, audio = detector.push(b"n", index in {20, 50, 80})
            if audio is not None:
                completed = audio
        self.assertTrue(completed)

    def test_short_false_trigger_is_discarded(self):
        detector = UtteranceDetector()
        for _ in range(6):
            detector.push(b"v", True)
        completed = None
        # Four initial silent frames complete the 10-frame start window, then
        # 100 more are required for the configured three-second endpoint.
        for _ in range(104):
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
        voice.play(Stream())
        self.assertEqual(writes, [b"\x01\x00", b"\x02\x00"])


if __name__ == "__main__":
    unittest.main()
