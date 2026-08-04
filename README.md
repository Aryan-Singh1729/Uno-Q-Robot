# Scout robot simulator

An English voice-and-vision robot simulator. When speech begins, Scout sends a
fresh webcam frame to Groq Qwen for a compact scene report while recording
continues. After transcription, GPT-OSS-120B receives only the transcript and
scene report, then simulates validated `move` and `turn` calls in the terminal.

This version must not be connected to motors. A single webcam image cannot
provide safe physical distance or angle estimates.

## Setup (Linux)

Install PortAudio using your distribution package manager (for Debian/Ubuntu,
`sudo apt install libportaudio2`), then create a Python 3.13+ environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
```

Put your Groq and Deepgram API keys in `.env`. Groq runs Whisper, Qwen vision,
and the text-only GPT-OSS agent; Deepgram Aura-2 streams speech output from one
request per reply. Scout listens while network requests and actions run.
New speech cancels the active turn and pending motion immediately. Microphone
processing is disabled only while Scout speaks, so its own output is ignored.

Model overrides are `VISION_MODEL`, `LLM_MODEL`, and `DEEPGRAM_TTS_MODEL`.

List devices and configure `MIC_DEVICE` / `OUTPUT_DEVICE` in `.env` if needed:

```bash
python main.py --list-devices
```

Run:

```bash
python main.py
```

Press `Ctrl+C` to stop. The exact JPEG sent with each LLM request overwrites
`latest-frame.jpg` in the project directory; it is ignored by Git. Audio and
generated speech stay in memory and are not retained.

## Runtime device controls

The `.env` values are startup defaults. Enter these commands in the running
terminal to test or switch sources without restarting; runtime changes are not
written back to `.env`:

```text
/devices
/status
/mic 4
/output 9
/camera 1
/test-mic 3
/calibrate-mic
/test-camera
/help
/quit
```

Audio devices may be selected by numeric ID or a unique name fragment. Use
`/mic default` or `/output default` to return to the operating-system default.
`/test-mic` records only for the requested duration, prints signal/VAD metrics,
transcribes the in-memory sample with Whisper, and does not save it.
At startup and after `/mic` changes, stay quiet during the 1.5-second calibration;
the resulting energy gate prevents steady background noise from holding an
utterance open. Run `/calibrate-mic` again whenever the room or mic gain changes.

## Motion conventions

- `move`: speed 1–100%, distance -500–500 cm; positive is forward.
- `turn`: speed 1–100%, angle -360–360 degrees; positive is left.
- Zero distance/angle is rejected.
- At most four motion calls are allowed per utterance.

Every valid action is spoken before it is simulated. Speaking during that
announcement does not interrupt Scout; microphone capture resumes before motion
starts. Speech detected during motion sets its stop event and supersedes the old
turn. The terminal simulation is instantaneous; a future motor implementation
must poll the supplied stop event and brake safely.

## Tests

The tests use mocked Groq, Deepgram, and hardware objects:

```bash
python -m unittest discover -v
```

## UNO Q later

Keep the Python agent on the UNO Q Debian MPU. Once the motor driver, encoders,
wheel geometry, proximity sensing, and emergency stop are defined, replace the
two simulator functions with Arduino Bridge calls to a deterministic MCU sketch.
