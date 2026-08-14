# Optimus UNO Q timed-motion voice robot - v14

This is one Arduino App Lab application containing both UNO Q processors:

- `python/main.py` launches microphone capture, transcription, autonomous tool calling,
  camera inspection, the live Web UI, and speech output on the Linux MPU.
- `sketch/sketch.ino` runs deterministic TB6612 motor control on the STM32 MCU.
- Arduino App Lab Bridge connects them. No SSH launch or separate Arduino upload is used.

## Observe-think-act mode

The HC-SR04, VL53L0X, MPU6050, and YDLIDAR X2 integrations are not loaded as motion guards in
this build. Disconnected or stale sensors cannot block a command. Direct forward/backward and
left/right instructions are parsed deterministically instead of being inferred from conversation
history. Find-and-move tasks run in a persistent camera loop that searches, aligns, approaches,
and rechecks the goal after every short timed action. The MCU still stops if Python status polling
disappears for 750 ms, and `/stop` remains available as an emergency brake.

```text
microphone -> complete utterance -> Whisper transcript -> observe/plan
           -> inspect camera -> move/turn/spin -> inspect again
           -> silent validated motion -> Bridge RPC
           -> TB6612 motor outputs
```

## Hardware safety

- Raise the wheels for the first motor test.
- Power the motors from a suitable external motor supply, not the UNO Q 3.3 V rail.
- Join the motor-supply ground, both TB6612 grounds, and UNO Q ground.
- Tie both TB6612 `STBY` inputs to D2, as required by the sketch.
- Leave the X2 disconnected for this motor-only test.
- Keep a physical power disconnect within reach.

## App layout

```text
uno-q-robot/
|-- app.yaml
|-- python/
|   |-- main.py
|   `-- requirements.txt
|-- sketch/
|   |-- sketch.ino
|   `-- sketch.yaml
|-- main.py
|-- audio_io.py
|-- live_view.py
|-- assets/
|-- robot_agent.py
|-- execute_motion_bridge.py
|-- .env
`-- ...
```

The root modules are deployed with the app and can also be unit-tested on a development PC.

## Configuration

Create `.env` in the imported App Lab app and set your own keys. Never export or share it.

```env
GROQ_API_KEY=gsk_replace_me
DEEPGRAM_API_KEY=replace_me

ROBOT_NAME=Scout
LLM_MODEL=openai/gpt-oss-120b
VISION_MODEL=qwen/qwen3.6-27b
STT_MODEL=whisper-large-v3-turbo
DEEPGRAM_TTS_MODEL=aura-2-thalia-en
PLAYBACK_GAIN=3.0

CAMERA_INDEX=0
# MIC_DEVICE=0
# OUTPUT_DEVICE=1

# Sensor and LiDAR motion gating is not loaded in v14.
```

## Import and run

1. Stop and delete the previous app so its build cache is not reused.
2. Import the newest ZIP from this repository.
3. Recreate `.env` with your private keys. You do not need to connect the LiDAR.
4. Connect the UNO Q, powered hub, EMEET SmartCam, and audio output.
5. Click **Run** once and let both sides finish building.
6. Confirm these messages appear:

```text
[MCU] timed-motion-v14 Bridge ready; timed drive enabled
[MOTOR] UNO Q MCU bridge is ready
[WEB] live camera view: http://arduinoq.local:7000
[AUDIO] ... EMEET SmartCam ... -> selected
```

The Python app requires the exact `timed-motion-v14` MCU handshake. If App Lab leaves an older
sensor sketch flashed, startup fails explicitly instead of silently using stale firmware.

For a direct test, keep the wheels raised and say:

```text
Move forward for five seconds at twenty percent speed.
```

Expected motion logs are:

```text
[PROPOSED] move(speed=20, seconds=5)
[STATE] acting
[MOTOR] move started
[MOTOR] motion ... finished: completed
```

In this build a disconnected X2 does not appear at startup and cannot block movement.

For an autonomous test, put the robot on a clear floor and say:

```text
Find a shoe in this room and move toward it.
```

The controller inspects a frame, rotates in short timed increments while searching, centers the target,
and approaches in 0.75-second steps (0.35 seconds when it looks large), taking a fresh frame after every
action. It speaks only three milestones: searching, target found/approaching, and target reached.
Open `http://arduinoq.local:7000` in a browser while the app runs to see the shared live camera
feed, task state, and emergency-stop button. The App Lab globe is only network/device status.

## Audio and camera behavior

Speech playback uses maximum clean peak normalization and requests 100% system mixer volume by
default. Each TTS response is buffered and resampled as one continuous waveform to avoid gaps.
The app prefers
EMEET/SmartCam microphones over generic USB audio devices. Audio received at
48 kHz is resampled outside PortAudio's real-time callback to prevent input-overflow storms.
A short post-playback cooldown prevents the robot hearing its own milestone. Microphone capture
is also closed during each physical action, preventing motor and gearbox noise from being
misclassified as an interrupt.

Vision tries each `/dev/video*` node if the configured camera opens but returns no frame. The
same camera object supplies both Qwen inspections and the Web UI feed, avoiding two processes
fighting over the USB camera.

## Controller envelope and calibration

- `move`: speed 1-100%, signed duration up to 120 seconds; positive is forward.
- `turn`: speed 1-100%, signed angle up to 3600 degrees; positive is left.
- `spin`: speed 1-100%, signed duration up to 120 seconds; positive is left.
- A target mission can use up to 240 short actions or 20 minutes.

The sensor/obstacle refusal path is absent. The finite task containment, 750 ms MCU connection
watchdog, Web UI emergency stop, App Lab Stop button, and physical power removal remain because
they prevent a lost controller from leaving powered motors running.

Linear distance estimation has been removed because there are no wheel encoders. A five-second
move is now scheduled directly as 5000 ms on the MCU, independent of speed. Angular `turn`
commands still use `DEG_PER_SEC_AT_100`; prefer `spin` when an exact rotation duration matters.
The observed front-right and rear-left wheel wiring polarity is already set to `-1` in the sketch.

## Tests

```bash
python -m unittest discover -v
```

Desktop tests mock network, camera, audio, LiDAR data, and Bridge operations. Physical motion
is enabled only inside Arduino App Lab after the versioned MCU readiness RPC succeeds.
