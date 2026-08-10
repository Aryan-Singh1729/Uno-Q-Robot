"""Microphone segmentation and streamed PCM playback."""

from __future__ import annotations

import io
import math
import queue
import threading
import time
import wave
from array import array
from collections import deque
from collections.abc import Callable

import numpy as np

SAMPLE_RATE = 16_000
FRAME_MS = 30
FRAME_SAMPLES = SAMPLE_RATE * FRAME_MS // 1_000
PRE_ROLL_FRAMES = 600 // FRAME_MS
START_WINDOW_FRAMES = 10
START_VOICED_FRAMES = 6
END_SILENCE_FRAMES = 3_000 // FRAME_MS
END_SPEECH_RUN_FRAMES = 3
MIN_SPEECH_FRAMES = 300 // FRAME_MS
MAX_UTTERANCE_FRAMES = 60_000 // FRAME_MS
INPUT_RATE_CANDIDATES = (SAMPLE_RATE, 48_000, 44_100, 32_000, 8_000)


class UtteranceDetector:
    """Turn a stream of VAD decisions into complete PCM utterances."""

    def __init__(
        self,
        *,
        pre_roll_frames: int = PRE_ROLL_FRAMES,
        start_window_frames: int = START_WINDOW_FRAMES,
        start_voiced_frames: int = START_VOICED_FRAMES,
        end_silence_frames: int = END_SILENCE_FRAMES,
        min_speech_frames: int = MIN_SPEECH_FRAMES,
        max_utterance_frames: int = MAX_UTTERANCE_FRAMES,
    ) -> None:
        self.pre_roll_frames = pre_roll_frames
        self.start_window_frames = start_window_frames
        self.start_voiced_frames = start_voiced_frames
        self.end_silence_frames = end_silence_frames
        self.min_speech_frames = min_speech_frames
        self.max_utterance_frames = max_utterance_frames
        self.reset()

    def reset(self) -> None:
        self.recording = False
        self._pre_roll: deque[tuple[bytes, bool]] = deque(maxlen=self.pre_roll_frames)
        self._start_window: deque[bool] = deque(maxlen=self.start_window_frames)
        self._frames: list[bytes] = []
        self._voiced_frames = 0
        self._silent_frames = 0
        self._voiced_run = 0

    def push(self, frame: bytes, voiced: bool) -> tuple[bool, bytes | None]:
        """Return (just_started, completed_pcm); b"" means a discarded clip."""
        if not self.recording:
            self._pre_roll.append((frame, voiced))
            self._start_window.append(voiced)
            if (
                len(self._start_window) == self.start_window_frames
                and sum(self._start_window) >= self.start_voiced_frames
            ):
                self.recording = True
                self._frames = [item[0] for item in self._pre_roll]
                self._voiced_frames = sum(item[1] for item in self._pre_roll)
                self._silent_frames = 0
                self._voiced_run = 1 if voiced else 0
                return True, None
            return False, None

        self._frames.append(frame)
        if voiced:
            self._voiced_frames += 1
            self._voiced_run += 1
            if self._voiced_run >= END_SPEECH_RUN_FRAMES:
                self._silent_frames = 0
            else:
                # Ignore isolated VAD spikes instead of restarting the endpoint timer.
                self._silent_frames += 1
        else:
            self._voiced_run = 0
            self._silent_frames += 1

        endpoint_reached = not voiced and self._silent_frames >= self.end_silence_frames
        if not endpoint_reached and len(self._frames) < self.max_utterance_frames:
            return False, None

        audio = b"".join(self._frames) if self._voiced_frames >= self.min_speech_frames else b""
        self.reset()
        return False, audio


def pcm_to_wav(pcm: bytes) -> bytes:
    output = io.BytesIO()
    with wave.open(output, "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(SAMPLE_RATE)
        wav.writeframes(pcm)
    return output.getvalue()


def pcm_rms(pcm: bytes) -> float:
    samples = array("h", pcm)
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples)) if samples else 0


def resample_pcm(
    pcm: bytes,
    *,
    channels: int,
    dtype: str,
    source_rate: int,
    target_rate: int,
) -> bytes:
    """Linearly resample interleaved PCM for output devices with fixed rates."""
    if source_rate == target_rate or not pcm:
        return pcm
    samples = np.frombuffer(pcm, dtype=dtype).reshape(-1, channels)
    target_length = max(1, round(len(samples) * target_rate / source_rate))
    source_positions = np.arange(len(samples))
    target_positions = np.linspace(0, len(samples) - 1, target_length)
    converted = np.column_stack(
        [np.interp(target_positions, source_positions, samples[:, channel]) for channel in range(channels)]
    )
    limits = np.iinfo(np.dtype(dtype))
    return np.clip(np.rint(converted), limits.min, limits.max).astype(dtype).tobytes()


def amplify_pcm(pcm: bytes, gain: float) -> bytes:
    """Apply gain to mono int16 PCM while clipping instead of wrapping."""
    if not pcm or gain == 1.0:
        return pcm
    samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
    return np.clip(np.rint(samples * gain), -32768, 32767).astype(np.int16).tobytes()


def _device(value: str | None) -> int | str | None:
    if value is None or not value.strip():
        return None
    return int(value) if value.lstrip("-").isdigit() else value


class VoiceIO:
    """Real microphone capture and WAV playback."""

    def __init__(
        self,
        input_device: str | None = None,
        output_device: str | None = None,
        playback_gain: float = 1.4,
    ) -> None:
        try:
            import sounddevice
            import webrtcvad
        except ImportError as exc:
            raise RuntimeError(
                "Audio dependencies are missing. Run: pip install -r requirements.txt"
            ) from exc
        self.sd = sounddevice
        # Mode 2 is still noise-resistant, but is less likely than mode 3 to
        # reject speech from the small microphones built into USB webcams.
        self.vad = webrtcvad.Vad(2)
        self.input_device: int | str | None = None
        self.input_rate = SAMPLE_RATE
        self.output_device = _device(output_device)
        if not 0.1 <= playback_gain <= 3.0:
            raise ValueError("playback gain must be between 0.1 and 3.0")
        self.playback_gain = playback_gain
        self.speech_rms_threshold = 120.0
        self.set_input_device(input_device)
        self._print_input_summary()

    @staticmethod
    def list_devices() -> None:
        try:
            import sounddevice
        except ImportError as exc:
            raise RuntimeError("sounddevice is not installed") from exc
        print(sounddevice.query_devices())

    def set_input_device(self, value: str | None) -> str:
        candidate = _device(value)
        if candidate is None:
            candidate = self._automatic_input_device()
        info = self.sd.query_devices(candidate, "input")
        self.input_rate = self._supported_input_rate(candidate, info)
        self.input_device = candidate
        return str(info["name"])

    def _automatic_input_device(self) -> int | str | None:
        devices = self.sd.query_devices()
        try:
            default_input = int(self.sd.default.device[0])
        except (AttributeError, IndexError, TypeError, ValueError):
            default_input = -1

        candidates: list[tuple[int, int]] = []
        for index, info in enumerate(devices):
            if int(info.get("max_input_channels", 0)) < 1:
                continue
            name = str(info.get("name", "")).lower()
            score = 10 if index == default_input else 0
            if "usb" in name:
                score += 50
            if "webcam" in name or "camera" in name or "uvc" in name:
                score += 45
            if "smartcam" in name or "emeet" in name:
                score += 60
            if "microphone" in name or " mic" in f" {name}":
                score += 35
            candidates.append((score, index))

        if not candidates:
            raise RuntimeError("no microphone input devices are visible to the UNO Q")
        return max(candidates)[1]

    def _supported_input_rate(self, device: int | str | None, info: object) -> int:
        default_rate = round(float(info["default_samplerate"]))  # type: ignore[index]
        rates = tuple(dict.fromkeys((*INPUT_RATE_CANDIDATES, default_rate)))
        last_error: Exception | None = None
        for rate in rates:
            try:
                self.sd.check_input_settings(
                    device=device,
                    channels=1,
                    dtype="int16",
                    samplerate=rate,
                )
                return rate
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"microphone does not support PCM capture: {last_error}")

    def _print_input_summary(self) -> None:
        print("[AUDIO] input devices visible to UNO Q:")
        for index, info in enumerate(self.sd.query_devices()):
            if int(info.get("max_input_channels", 0)) > 0:
                marker = " -> selected" if index == self.input_device else ""
                print(f"[AUDIO]   {index}: {info['name']}{marker}")
        selected = self.sd.query_devices(self.input_device, "input")
        print(f"[AUDIO] capturing from {selected['name']} at {self.input_rate} Hz")

    def set_output_device(self, value: str | None) -> str:
        candidate = _device(value)
        info = self.sd.query_devices(candidate, "output")
        self.output_device = candidate
        return str(info["name"])

    def describe_devices(self) -> tuple[str, str]:
        input_name = str(self.sd.query_devices(self.input_device, "input")["name"])
        output_name = str(self.sd.query_devices(self.output_device, "output")["name"])
        return input_name, output_name

    def calibrate_input(self, seconds: float = 1.5) -> float:
        levels = sorted(pcm_rms(frame) for frame in self._fixed_capture(seconds))
        percentile_90 = levels[round((len(levels) - 1) * 0.9)]
        self.speech_rms_threshold = min(3_000.0, max(120.0, percentile_90 * 2.2))
        return self.speech_rms_threshold

    def _capture_callback(self, indata: bytes) -> bytes:
        pcm = bytes(indata)
        if self.input_rate != SAMPLE_RATE:
            pcm = resample_pcm(
                pcm,
                channels=1,
                dtype="int16",
                source_rate=self.input_rate,
                target_rate=SAMPLE_RATE,
            )
        return pcm

    def _fixed_capture(self, seconds: float) -> list[bytes]:
        frames: queue.Queue[bytes] = queue.Queue()

        def callback(indata: bytes, _count: int, _time: object, status: object) -> None:
            if status:
                print(f"[AUDIO] {status}")
            # Keep PortAudio's real-time callback minimal. Resampling here can
            # starve 48 kHz USB devices and produce repeated input overflows.
            frames.put(bytes(indata))

        captured: list[bytes] = []
        frame_count = max(1, math.ceil(seconds * 1_000 / FRAME_MS))
        with self.sd.RawInputStream(
            samplerate=self.input_rate,
            blocksize=self.input_rate * FRAME_MS // 1_000,
            channels=1,
            dtype="int16",
            device=self.input_device,
            callback=callback,
        ):
            for _ in range(frame_count):
                captured.append(self._capture_callback(frames.get(timeout=1)))
        return captured

    def _is_speech(self, frame: bytes) -> bool:
        return (
            pcm_rms(frame) >= self.speech_rms_threshold
            and self.vad.is_speech(frame, SAMPLE_RATE)
        )

    def test_input(self, seconds: float = 3.0) -> tuple[dict[str, float], bytes]:
        """Measure a fixed sample and return metrics plus an in-memory WAV."""
        captured = self._fixed_capture(seconds)
        frame_count = len(captured)
        voiced = 0
        peak = 0
        square_sum = 0
        sample_count = 0
        for frame in captured:
            voiced += self._is_speech(frame)
            samples = array("h", frame)
            if samples:
                peak = max(peak, max(abs(sample) for sample in samples))
                square_sum += sum(sample * sample for sample in samples)
                sample_count += len(samples)
        return (
            {
                "seconds": frame_count * FRAME_MS / 1_000,
                "voiced_percent": voiced * 100 / frame_count,
                "rms": math.sqrt(square_sum / sample_count) if sample_count else 0,
                "peak": float(peak),
            },
            pcm_to_wav(b"".join(captured)),
        )

    def capture(
        self,
        *,
        on_ready: Callable[[], None] | None = None,
        on_speech_start: Callable[[], None] | None = None,
        stop_event: threading.Event | None = None,
    ) -> bytes | None:
        stop_event = stop_event or threading.Event()
        frames: queue.Queue[bytes] = queue.Queue(maxsize=400)
        detector = UtteranceDetector()

        def callback(indata: bytes, _count: int, _time: object, status: object) -> None:
            if status:
                print(f"[AUDIO] {status}")
            try:
                frames.put_nowait(bytes(indata))
            except queue.Full:
                pass

        with self.sd.RawInputStream(
            samplerate=self.input_rate,
            blocksize=self.input_rate * FRAME_MS // 1_000,
            channels=1,
            dtype="int16",
            device=self.input_device,
            callback=callback,
        ):
            if on_ready:
                on_ready()
            diagnostic_started = time.monotonic()
            diagnostic_peak = 0
            diagnostic_rms = 0.0
            while not stop_event.is_set():
                try:
                    frame = self._capture_callback(frames.get(timeout=0.1))
                except queue.Empty:
                    continue
                samples = array("h", frame)
                if samples:
                    diagnostic_peak = max(diagnostic_peak, max(abs(sample) for sample in samples))
                    diagnostic_rms = max(diagnostic_rms, pcm_rms(frame))
                if time.monotonic() - diagnostic_started >= 5:
                    print(
                        f"[AUDIO] waiting for speech: peak={diagnostic_peak}, "
                        f"max RMS={diagnostic_rms:.0f}, threshold={self.speech_rms_threshold:.0f}"
                    )
                    if diagnostic_peak < 20:
                        print(
                            "[AUDIO] no microphone signal; set MIC_DEVICE to the USB "
                            "camera input ID shown above"
                        )
                    diagnostic_started = time.monotonic()
                    diagnostic_peak = 0
                    diagnostic_rms = 0.0
                voiced = self._is_speech(frame)
                started, audio = detector.push(frame, voiced)
                if started and on_speech_start:
                    on_speech_start()
                if audio is None:
                    continue
                if audio:
                    return pcm_to_wav(audio)
                # A short false trigger was discarded; keep listening.
        return None

    def play(self, stream: object) -> None:
        """Play Deepgram's 24 kHz mono int16 PCM response as it arrives."""
        source_rate = 24_000
        output_rate = source_rate
        try:
            self.sd.check_output_settings(
                device=self.output_device,
                channels=1,
                dtype="int16",
                samplerate=output_rate,
            )
        except self.sd.PortAudioError:
            output_rate = round(
                self.sd.query_devices(self.output_device, "output")["default_samplerate"]
            )
            print(f"[AUDIO] resampling playback from {source_rate}Hz to {output_rate}Hz")

        output = None
        for attempt in range(2):
            try:
                output = self.sd.RawOutputStream(
                    samplerate=output_rate,
                    channels=1,
                    dtype="int16",
                    device=self.output_device,
                )
                break
            except self.sd.PortAudioError:
                if attempt:
                    raise
                time.sleep(0.2)

        reader = getattr(stream, "read1", stream.read)
        pending = b""
        with output:
            while chunk := reader(2_400):
                pcm = pending + chunk
                aligned = len(pcm) - len(pcm) % 2
                pending = pcm[aligned:]
                pcm = pcm[:aligned]
                if output_rate != source_rate:
                    pcm = resample_pcm(
                        pcm,
                        channels=1,
                        dtype="int16",
                        source_rate=source_rate,
                        target_rate=output_rate,
                    )
                pcm = amplify_pcm(pcm, self.playback_gain)
                if pcm:
                    output.write(pcm)
        if pending:
            raise RuntimeError("Deepgram returned an incomplete PCM sample")
