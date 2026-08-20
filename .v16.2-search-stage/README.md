# WALL-E UNO Q LiDAR-guarded robot - v16.1

This is one Arduino App Lab application containing both UNO Q processors:

- `python/main.py` launches microphone capture, transcription, autonomous tool calling,
  camera inspection, the live Web UI, and speech output on the Linux MPU.
- `sketch/sketch.ino` runs deterministic TB6612 motor control and the YDLIDAR X2
  emergency guard on the STM32 MCU.
- Arduino App Lab Bridge connects them. No SSH launch or separate Arduino upload is used.

## LiDAR emergency guard

This build focuses only on the YDLIDAR X2. Linux reads it through the X2 USB-to-UART adapter and
checks the scan before starting a command and every 50 ms while motors run. Any fresh point
anywhere in the 360-degree scan at **100 mm or closer** sends `stop_robot` through the Bridge,
which brakes all four motors. The configured forward sector is reported separately for navigation.
Motion is rejected or stopped when the required LiDAR scan is missing or stale. HC-SR04,
VL53L0X, and MPU6050 remain disabled. The MCU's independent 750 ms Python-controller watchdog
still stops motion if Linux or the Bridge stops responding.

```text
microphone -> FIFO utterance queue -> Whisper transcript -> observe/plan
           -> inspect camera -> move/timed turn -> inspect again
           -> silent validated motion -> Bridge RPC
           -> TB6612 motor outputs
YDLIDAR X2 -> USB serial adapter -> Linux 10 cm guard -> Bridge -> four-motor brake
```

## Hardware safety

- Raise the wheels for the first motor test.
- Power the motors from a suitable external motor supply, not the UNO Q 3.3 V rail.
- Join the motor-supply ground, both TB6612 grounds, and UNO Q ground.
- Tie both TB6612 `STBY` inputs to D2, as required by the sketch.
- Connect the X2 to its USB-to-UART/motor-control adapter and connect that adapter to the UNO Q
  Linux USB host through the powered hub.
- Do **not** connect the X2 data wire to D0/D1 in this App Lab build. UNO Q reserves `Serial1`
  on those pins for `Arduino_RouterBridge`; sharing it would break motor RPC commands.
- Power the X2/adapter from its specified 5 V source and confirm the scanner is rotating.
- Mount the X2 with its zero-degree direction facing the front of the chassis. If the map appears
  rotated, adjust `LIDAR_FRONT_ANGLE_DEG` in `sketch/sketch.ino` and rebuild.
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
OpenCV is intentionally absent from `python/requirements.txt`: Arduino App Lab's ARM64 base
image already supplies its custom OpenCV build. Requesting it again makes App Lab 0.9.0 try to
resolve an internal build tag from the public package index and abort all Python provisioning.

## Configuration

Create `.env` in the imported App Lab app and set your own keys. Never export or share it.

```env
GROQ_API_KEY=replace_me
DEEPGRAM_API_KEY=replace_me

ROBOT_NAME=WALL-E
LLM_MODEL=openai/gpt-oss-120b
VISION_MODEL=qwen/qwen3.6-27b
STT_MODEL=whisper-large-v3-turbo
DEEPGRAM_TTS_MODEL=aura-2-aries-en
PLAYBACK_GAIN=3.0

CAMERA_INDEX=0
# MIC_DEVICE=7
# OUTPUT_DEVICE=0

# LIDAR_PORT=/dev/ydlidar
# The emergency threshold is fixed at exactly 100 mm in lidar_x2.py.
```

Text planning, Whisper transcription, and Qwen vision all use Groq. The main text model is
`openai/gpt-oss-120b`, which supports the local motion and camera tool schemas used by this app.

## Import and run

1. Stop and delete the previous app so its build cache is not reused.
2. Import the newest ZIP from this repository.
3. Recreate `.env` with your private keys.
4. Connect the X2 USB serial adapter through the powered USB hub, plus the EMEET SmartCam and
   audio output. Do not use D0/D1 for the X2 while RouterBridge is enabled.
5. Click **Run** once and let both sides finish building.
6. Confirm these messages appear:

```text
[MCU] lidar-guard-v16.0 Bridge ready; Serial1 reserved for RouterBridge
[LIDAR] opened YDLIDAR X2 serial port /dev/ttyUSB... at 115200 baud
[MOTOR] UNO Q MCU bridge is ready
[LIDAR] Linux USB emergency guard configured at 10.0 cm
[WEB] live camera + LiDAR view: http://arduinoq.local:7000
[AUDIO] ... EMEET SmartCam ... -> selected
```

The Python app requires the exact `lidar-guard-v16.0` MCU handshake. If App Lab leaves an older
sensor sketch flashed, startup fails explicitly instead of silently using stale firmware.

For a direct test, keep the wheels raised and say:

```text
Move forward for five seconds at twenty percent speed.
```

Expected motion logs are:

```text
[PROPOSED] move(speed=20, duration_seconds=5)
[STATE] acting
[MOTOR] move started
[MOTOR] motion ... finished: completed
```

Before testing the motors, type `/lidar` in the App Lab Python console. A healthy result has
`"connected": true`, `"scan_fresh": true`, and a rising `packet_count`. Motion remains blocked
if those fields are false.

For an autonomous test, put the robot on a clear floor and say:

```text
Find a shoe in this room and move toward it.
```

The controller inspects a frame, rotates in short timed increments while searching, centers the target,
and approaches in 0.75-second steps (0.35 seconds when it looks large), taking a fresh frame after every
action. It speaks only three milestones: searching, target found/approaching, and target reached.
Open the **Network URL printed by WebUI** (or `http://arduinoq.local:7000`) while the app runs.
The page shows the camera, a live polar LiDAR map, front and nearest-obstacle distances,
nearest-obstacle angle, connection/freshness state,
the fixed red 10 cm boundary, task state, and the manual emergency-stop button. The App Lab globe
is only network/device status.

## Test the 10 cm stop

1. Raise the wheels first and keep the physical power cutoff within reach.
2. Start the app and confirm `/lidar` reports a fresh scan.
3. Put a flat, opaque target in front of the LiDAR and verify the live map orientation.
4. Command a short, low-speed motion.
5. Move the target into the scan plane from any direction at 10 cm. The MCU log and motion result should report
   `YDLIDAR X2 emergency stop`, and all four motor outputs should brake.

The X2's documented measurement range begins at 0.10 m, so 10 cm is the edge of its specified
range. This build follows the requested threshold exactly and does not stop for readings above
100 mm. Readings below the sensor's reliable range cannot be guaranteed by software.

## Audio and camera behavior

Speech playback uses maximum clean peak normalization and requests 100% system mixer volume by
default. TTS PCM is played incrementally as network chunks arrive, reducing time to first audio.
The app prefers
EMEET/SmartCam microphones over generic USB audio devices. Audio received at
48 kHz is resampled outside PortAudio's real-time callback to prevent input-overflow storms.
A short post-playback cooldown prevents the robot hearing its own milestone. Completed commands
are processed in FIFO order; newer speech does not cancel the active turn. Microphone capture is
also closed during each physical action so motor and gearbox noise is not queued as a command.
If the USB camera temporarily disconnects, the app refreshes PortAudio, reselects the microphone,
and retries without terminating. Recovery follows the original microphone by its stable USB name,
even when Linux assigns it a different ALSA card number; it will not silently switch from the EMEET
microphone to an unrelated USB sound card. Repeated PortAudio input overflows close and reopen the
stream once instead of flooding the callback indefinitely. Camera-node scans use exponential backoff
up to 10 seconds, and a newly enumerated UVC camera gets three warm-up reads before it is rejected.

Vision tries each `/dev/video*` node if the configured camera opens but returns no frame. The
same camera object supplies both Qwen inspections and the Web UI feed, avoiding two processes
fighting over the USB camera.

## Controller envelope and calibration

- `move`: speed 1-100%, signed duration up to 60 seconds; positive is forward.
- `turn`: fixed 50% speed, signed duration up to 60 seconds; positive is left.
- The internal `spin` primitive remains available to persistent camera missions.
- A general LLM turn can use at most four motion calls; inspections do not consume that count.
- A target mission can use up to 240 short actions or 20 minutes.

The sensor/obstacle refusal path is absent. The finite task containment, 750 ms MCU connection
watchdog, Web UI emergency stop, App Lab Stop button, and physical power removal remain because
they prevent a lost controller from leaving powered motors running.

Linear distance estimation has been removed because there are no wheel encoders. A five-second
move is scheduled directly as 5000 ms on the MCU, independent of speed. A user-facing `turn`
is also duration-based and is translated to the existing MCU `spin_robot` RPC at fixed 50% speed.
The physical channel map verified by the v15.1 wheel observations is Driver 1 A = front-left,
Driver 1 B = rear-left, Driver 2 A = front-right, and Driver 2 B = rear-right. The corresponding
polarities are front-left `-1`, rear-left `-1`, front-right `+1`, and rear-right `+1`. This rear
ownership correction leaves the already-working forward/backward electrical outputs unchanged.

The expected physical wheel directions are:

| Command | Front left | Rear left | Front right | Rear right |
| --- | --- | --- | --- | --- |
| Forward | forward | forward | forward | forward |
| Backward | backward | backward | backward | backward |
| Left | forward | forward | backward | backward |
| Right | backward | backward | forward | forward |

The final floor test showed that the assembled chassis turns opposite to the raw tank-turn sign,
so v15.4 inverts only the requested turn direction at the MCU boundary. Channel ownership,
individual motor polarity, and the already-correct forward/backward outputs are unchanged.

## Tests

```bash
python -m unittest discover -v
```

Desktop tests mock network, camera, audio, LiDAR data, and Bridge operations. Physical motion
is enabled only inside Arduino App Lab after the versioned MCU readiness RPC succeeds.
