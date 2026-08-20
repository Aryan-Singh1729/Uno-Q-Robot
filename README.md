# WALL-E UNO Q robot - v18.0 and ROS 2 migration

This repository contains the current Arduino App Lab application for a four-wheel
Arduino UNO Q robot. It runs one STM32 sketch and one Linux Python application as
a single App Lab app.

The current release is `uno-q-robot-app-v18.0-final-navigation.zip`. It provides
voice commands, on-demand camera vision, YDLIDAR X2 local navigation, a live WebUI,
local target following, a front ultrasonic emergency stop, MPU6050 correction,
and real motor control through Arduino RouterBridge.

For the complete history, solved problems, UNO Q inspection, and future ROS plan,
read [Context.md](Context.md).

## Current architecture

```text
Voice -> STT -> direct command parser / LLM -> validated high-level action
                                                        |
YDLIDAR X2 -> NavigationController ---------------------+
                                                        v
Camera -> target identification -> local tracker -> Bridge/RPC -> MCU -> motors
                                                        ^
Front ultrasonic + MPU6050 + watchdog ------------------+

WebUI :7000 <- live camera, LiDAR, sensors, events, follower state, emergency stop
```

LiDAR is the primary navigation sensor. The one front ultrasonic sensor is only a
direct-ahead emergency backup. The MPU6050 assists turning, heading correction,
turn-stall detection, and tilt stopping. Navigation and stopping do not wait for
the LLM, network, or speech.

The current controller uses timed motion because the motors have no encoders.
Commands use seconds and speed percentage, not centimeters or degrees.

## Final wiring

All grounds must be common.

### Motors

| Driver/channel | Motor | Control pins |
|---|---|---|
| TB6612 #1 A | Front left | PWMA D3, AIN1 D4, AIN2 D7 |
| TB6612 #1 B | Rear right | PWMB D5, BIN1 D8, BIN2 D12 |
| TB6612 #2 A | Front right | PWMA D6, AIN1 D13, AIN2 A0 |
| TB6612 #2 B | Rear left | PWMB D9, BIN1 A1, BIN2 A2 |
| Both TB6612 boards | STBY | D2 shared |

Both drivers use Buck #2 5 V for VCC and VM and common GND. Do not change this
mapping: it includes the final tested left/right correction.

### Sensors and USB devices

| Device | Signal | Connection |
|---|---|---|
| Front ultrasonic | VCC / GND | Buck #2 5 V / common GND |
| Front ultrasonic | TRIG / ECHO | A3 / A4 |
| MPU6050 | VCC / GND | Buck #2 5 V / common GND |
| MPU6050 | SDA / SCL | Physical SDA / physical SCL header pins |
| MPU6050 | XDA / XCL / INT | Not connected |
| MPU6050 | AD0 | Default (`0x68`) |
| YDLIDAR X2 | Data | Linux USB serial adapter |
| EMEET SmartCam C950 | Video and microphone | Linux USB |

There are no ToF sensors in the final software. The reverted standalone analog
microphone versions must not be used; current audio selection prefers the EMEET
camera microphone.

## Configuration

Copy `.env.example` to `.env` inside the imported App Lab application and replace
the placeholders. Never commit, upload, screenshot, or include the real `.env` in
a ZIP.

Important variables:

```dotenv
GROQ_API_KEY=gsk_replace_me,gsk_optional_backup
CEREBRAS_API_KEYS=
DEEPGRAM_API_KEY=replace_me
ROBOT_NAME=WALL-E
LLM_MODEL=openai/gpt-oss-120b
VISION_MODEL=qwen/qwen3.6-27b
STT_MODEL=whisper-large-v3-turbo
DEEPGRAM_TTS_MODEL=aura-2-thalia-en
PLAYBACK_GAIN=3.0
CAMERA_INDEX=0
# LIDAR_PORT=/dev/ydlidar
# MIC_DEVICE=EMEET SmartCam C950: USB Audio
# OUTPUT_DEVICE=1
```

Comma-separated API-key pools rotate only after HTTP 429. Any key previously
shown in chat or a screenshot should be revoked and replaced.

## Import and run in Arduino App Lab

1. Import `uno-q-robot-app-v18.0-final-navigation.zip` from **My Apps**.
2. Open the imported app and add its private `.env` values.
3. Keep the wheels raised for the first startup test.
4. Press **Run** in App Lab. Do not launch root `main.py` manually over SSH.
5. Confirm the log reports:
   - firmware `navigation-v18.0`
   - MCU Bridge ready
   - selected EMEET microphone
   - camera status
   - YDLIDAR serial port and fresh packets
   - the WebUI Local and Network URLs
6. Test `/status`, `/lidar`, `/sensors`, and `/test-camera` before placing the
   robot on the floor.

The ZIP intentionally contains both root `main.py` and `python/main.py`. App Lab
requires that packaging; removing or nesting either entry can cause "main python
file missing from app".

## Live camera and LiDAR view

Open the URL printed by the WebUI log, normally:

```text
http://arduinoq.local:7000
```

If `.local` name resolution does not work, use the printed Network URL such as
`http://<UNO-Q-IP>:7000`. The App Lab globe icon is not the camera viewer.

The page shows live camera frames, LiDAR scan data, navigation events, supplemental
sensor state, follower status, and an emergency-stop control. If the Python app
crashes or stops, port 7000 closes and the browser will report connection refused.

## Commands

Useful console commands:

```text
/help
/status
/lidar
/sensors
/test-camera
/follow person
/follow dog
/unfollow
/stop
```

Voice examples:

```text
Move forward for five seconds at twenty percent speed.
Move backward for two seconds at fifteen percent speed.
Turn left for one second.
Turn right for one second.
Find a shoe and move towards it.
Follow me.
Stop following.
```

Direct movement defaults to 20% speed. Public turns use a fixed 50% speed. Current
software rejects distance and angle commands because unencoded motors cannot
measure them accurately.

## Safety and local navigation

- LiDAR is required for physical forward navigation; missing or stale scans fail
  closed.
- The LiDAR hard emergency boundary is fixed at exactly 100 mm or less.
- A fresh front ultrasonic reading at 100 mm or less stops all active motion on
  the MCU.
- Excessive MPU tilt stops motion immediately.
- MPU yaw corrects straight-line drift and detects a turn that produced no
  measured rotation.
- The MCU watchdog stops motors when Python stops polling.
- Bridge errors, timeouts, explicit stops, and shutdown all issue motor stop.
- The LLM receives semantic navigation events, not continuous raw sensor values.

Use a controlled floor, begin with low speed and short durations, and keep access
to `/stop` or the WebUI emergency stop.

## Local follower behavior

For `follow me` or a named visible person/animal, vision identifies the target
once and seeds a local OpenCV tracker. The camera supplies bearing and LiDAR
supplies distance and obstacle geometry. The local loop controls short actions
without repeated LLM calls. If the target is lost, the robot stops and attempts
local reacquisition.

Human/animal following has not yet been rebuilt on Nav2; this describes the
current custom v18 follower.

## Development and verification

Run all desktop tests from the repository root:

```powershell
python -m unittest discover -v
```

At the 2026-08-20 handoff, all 97 tests pass. They cover audio overflow recovery,
camera re-enumeration, tool validation, direct direction parsing, Bridge polling,
the four-wheel truth table, LiDAR packet decoding and 10 cm guard, local obstacle
recovery, missions, follower behavior, WebUI publishing, and App Lab packaging.

The real sketch must still be compiled and flashed by App Lab because the UNO Q
Zephyr platform and RouterBridge libraries are managed there.

## Repository map

| Path | Responsibility |
|---|---|
| `main.py` | Linux application and command lifecycle |
| `python/main.py` | App Lab entry wrapper |
| `robot_agent.py` | STT/LLM/vision/TTS and tool validation |
| `audio_io.py` | microphone capture, recovery, VAD, playback |
| `lidar_x2.py` | X2 serial decoder and LiDAR guard |
| `navigation_controller.py` | deterministic local navigation |
| `local_follower.py` | camera/LiDAR target following |
| `execute_motion_bridge.py` | Python-to-MCU motor RPC |
| `live_view.py`, `assets/` | port 7000 telemetry UI |
| `sketch/sketch.ino` | real-time motor and supplemental safety layer |
| `sketch/sketch.yaml` | UNO Q sketch dependencies |
| `Context.md` | full handoff and migration decisions |

## Planned ROS 2/Nav2/SLAM migration

No ROS software has been installed or integrated yet. The current custom
navigation remains the running implementation.

The planned stack is:

```text
YDLIDAR X2 -> /scan -> RF2O laser odometry ----+
MPU6050 -> /imu/data ---------------------------+-> robot_localization
                                                    -> odom -> base_link
/scan + odometry -> SLAM Toolbox -> map -> odom
map + scan + odometry -> Nav2 -> /cmd_vel -> Bridge base driver -> MCU
```

Nav2 does not require encoder motors, but it does require valid continuous local
odometry. The current preferred no-encoder experiment is RF2O 2D LiDAR odometry
fused with the MPU6050 using `robot_localization`. This should be treated as a
slow indoor prototype; LiDAR odometry can fail during fast turns or in
feature-poor/dynamic scenes. MPU-only translation and command-time odometry are
not adequate replacements.

The UNO Q runs Debian 13 aarch64 and had about 1.1 GB free internal storage and
roughly 2 GB RAM at the last inspection. ROS 2 Jazzy should therefore run in a
minimal Ubuntu 24.04 ARM64 Docker container. Run RViz on the Windows PC, not on
the UNO Q.

Arduino supports microSD/USB storage through a powered USB-C hub. Use a 128 GB
high-endurance microSD or, preferably, a USB SSD formatted `ext4` for Docker data,
ROS workspaces, maps, builds, logs, and bags. External storage does not add RAM or
automatically enlarge `/`.

Before installing anything, the next phase must re-check that:

1. the powered USB hub and external storage enumerate on Linux;
2. the X2 appears as `/dev/ttyUSB*` or a stable `/dev/ydlidar` link;
3. storage is mounted persistently;
4. only one controller owns motor commands during migration.

See [Context.md](Context.md) for the exact staged migration plan and the previous
read-only UNO Q inspection results.
