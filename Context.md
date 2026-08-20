# WALL-E UNO Q robot: complete project handoff

Last updated: 2026-08-20

This document is the authoritative handoff for continuing the project in a new
chat. Read it before changing code. It records the project history, confirmed
hardware, current v18.0 implementation, problems already solved, known limits,
UNO Q inspection results, and the proposed ROS 2 direction.

Do not copy API keys or passwords from chat history into documentation or source
control. The real `.env` is private. Any credential that was exposed in a chat or
screenshot should be rotated.

## 1. Project objective

The robot is a four-wheel Arduino UNO Q platform named WALL-E. The UNO Q has two
execution sides:

- The STM32 MCU runs the Arduino sketch and performs real-time motor control,
  watchdog stopping, ultrasonic emergency stopping, and MPU6050 corrections.
- The Linux MPU runs Python for microphone capture, speech-to-text, LLM tool
  calling, camera vision, LiDAR parsing, local navigation, follower logic, TTS,
  and the live WebUI.
- Arduino `RouterBridge`/RPC connects Python to the sketch. Both sides are
  deployed and started together by Arduino App Lab. `main.py` must not be run as
  a separate SSH terminal process for normal operation.

The current stable deliverable is:

`uno-q-robot-app-v18.0-final-navigation.zip`

## 2. Final physical hardware and wiring

All grounds must be tied to one common ground. The current code intentionally
contains no VL53L0X ToF integration and no additional ultrasonic sensors.

### Motors and TB6612FNG drivers

The physical channel ownership below was verified through repeated four-wheel
tests and must not be rearranged.

| Driver/channel | Motor | UNO Q control pins | Driver output |
|---|---|---|---|
| TB6612 #1 A | Front left | PWMA D3, AIN1 D4, AIN2 D7 | AO1/AO2 |
| TB6612 #1 B | Rear right | PWMB D5, BIN1 D8, BIN2 D12 | BO1/BO2 |
| TB6612 #2 A | Front right | PWMA D6, AIN1 D13, AIN2 A0 | AO1/AO2 |
| TB6612 #2 B | Rear left | PWMB D9, BIN1 A1, BIN2 A2 | BO1/BO2 |
| Both drivers | Shared standby | STBY D2 | n/a |

Both drivers use Buck #2 5 V for VCC and VM and use common ground.

The tested logical-forward polarity constants in `sketch/sketch.ino` are:

```cpp
FRONT_LEFT_POLARITY  = -1
FRONT_RIGHT_POLARITY =  1
REAR_LEFT_POLARITY   = -1
REAR_RIGHT_POLARITY  =  1
```

For a semantic left tank turn, both left wheels reverse and both right wheels
advance. For a semantic right turn, both left wheels advance and both right
wheels reverse. Earlier releases had incorrect rear-wheel behavior and later had
left/right instructions inverted; the v18 truth table and tests include the
correct mapping.

### Front ultrasonic sensor

This is only a close-range, direct-ahead emergency backup. It is not the primary
navigation sensor.

| Signal | Connection |
|---|---|
| VCC | Buck #2 5 V |
| GND | Common GND |
| TRIG | A3 |
| ECHO | A4 |

The A3/A4 ordering came from the previously tested sensor code and must not be
guessed or swapped. The MCU stops all active motion when a fresh valid reading is
exactly 100 mm (10 cm) or less.

### MPU6050

| Signal | Connection |
|---|---|
| VCC | Buck #2 5 V |
| GND | Common GND |
| SDA | Physical SDA header pin on UNO Q |
| SCL | Physical SCL header pin on UNO Q |
| XDA, XCL, INT | Not connected |
| AD0 | Default, giving I2C address `0x68` |

The sketch uses `Wire.begin()` and the physical `SDA`/`SCL` symbols. Do not
describe or remap this connection as D20/D21. The current MCU code calibrates gyro
bias at startup, estimates yaw, roll, and pitch, corrects straight-line drift,
checks that turns produce rotation, and stops on excessive tilt.

### YDLIDAR X2

The YDLIDAR X2 is the primary navigation sensor. It belongs on the UNO Q Linux
side through its USB-to-serial adapter. The current custom Python reader searches
for an explicitly configured `LIDAR_PORT`, `/dev/ydlidar`, and `/dev/ttyUSB*`.
It uses 115200 baud and decodes the X2 triangle packets itself.

For the planned ROS driver, Linux must expose a serial device such as
`/dev/ttyUSB0`. If the LiDAR is wired only to an MCU UART, the official ROS driver
cannot access it without a custom MCU-to-ROS scan bridge. Direct Linux USB serial
is preferred.

### Camera, microphone, and speaker

- Camera: EMEET SmartCam C950 USB camera, with index fallback/re-enumeration.
- Microphone: use the EMEET camera's built-in USB microphone. The standalone
  analog microphone experiment was reverted because it performed worse and
  repeatedly overflowed.
- Speaker/output: selected through PortAudio/ALSA; `PLAYBACK_GAIN=3.0` is the
  current default. Deepgram TTS audio is streamed rather than buffered as a full
  response.

The v15.5/v15.6 external-microphone-only experiments are obsolete and should not
be restored. Current automatic device selection prefers a stable EMEET device
name rather than a numeric ALSA index, because USB indices can change.

## 3. Current v18.0 software architecture

```text
Microphone -> VAD/capture -> Groq Whisper STT -> direct parser or LLM
                                                  |
Camera -> Qwen vision on demand ------------------+
                                                  v
                                  validated high-level action
                                                  |
YDLIDAR X2 -> local sectors -> NavigationController
       |                       / corridor correction
       |                       / obstacle recovery
       +-> exact 10 cm guard -+-> Bridge/RPC -> MCU -> TB6612 -> motors
                                                   |
Front ultrasonic (10 cm) + MPU6050 + watchdog -----+

Camera identifies target once -> local OpenCV tracker --+
YDLIDAR range/sectors ------------------------------------+-> LocalFollower
                                                               -> local actions

WebUI :7000 <- camera frame + LiDAR points + sensors + events + emergency stop
```

The navigation loop acts immediately and does not wait for the LLM or TTS. Raw
sensor streams are not continuously sent to the LLM. Instead, the navigation
layer emits semantic events such as `OBSTACLE_DETECTED`, `OBSTACLE_AVOIDING`,
`WALL_TOO_CLOSE`, `HEADING_CORRECTION`, `TILT_WARNING`, `PATH_BLOCKED`, and
`ROBOT_STUCK`.

### Motion semantics

Distance commands were removed because plain BO motors have no encoders and
time/RPM calculations were too inaccurate. Public motion is duration-based:

- `move`: signed seconds and 1-100% speed. Positive is forward; negative is
  backward. Default direct-command speed is 20%.
- `turn`: signed seconds at a fixed 50% speed. Positive is left; negative is
  right. It uses the MCU `spin_robot` timed RPC.
- `spin`: internal duration-based primitive used by persistent target missions.

Direct unambiguous speech such as "move forward for five seconds at twenty
percent" is parsed locally before conversational history can redirect it. Angle
and centimeter commands are intentionally rejected and the user is asked for a
duration.

Forward movement is split into 0.35-second chunks by `NavigationController`.
Before and during each chunk, the controller checks fresh LiDAR sectors. It can
center away from a close corridor wall, reverse briefly, turn toward the clearer
side, and retry. Missing/stale required LiDAR fails closed for forward motion.

### MCU real-time behavior

The sketch exposes these safe Bridge methods:

- `move_robot(speed, seconds)`
- `turn_robot(speed, angleDegrees)` (retained MCU capability)
- `spin_robot(speed, seconds)`
- `stop_robot(reason)`
- `robot_status()`
- `read_sensors()`

The Python motion backend serializes Bridge calls, verifies firmware version
`navigation-v18.0`, polls every 50 ms, matches motion IDs, applies the LiDAR guard,
and sends a stop RPC on cancellation, bridge failure, timeout, or inconsistent
state.

The MCU independently provides:

- 750 ms Python controller watchdog
- non-blocking motion deadline
- 100 ms short electrical brake before releasing outputs
- front ultrasonic stop at <=100 mm
- tilt stop at approximately 28 degrees
- straight-drive yaw correction
- turn-stall detection
- STBY LOW and outputs released after braking

### Voice, LLM, vision, and TTS

- STT: Groq `whisper-large-v3-turbo`
- Default text LLM: `openai/gpt-oss-120b`; Cerebras keys/models are optional
- Vision: Groq `qwen/qwen3.6-27b`, called only when visual evidence is needed
- TTS: Deepgram `aura-2-thalia-en`
- Comma-separated API-key pools rotate only after HTTP 429 and stop after one
  exhausted cycle.
- The microphone is paused while WALL-E speaks and while motors are running, so
  speaker/motor noise does not become a new instruction.
- New utterances are queued FIFO; they do not silently cancel an active action.
  Explicit `/stop`, WebUI emergency stop, and shutdown do stop motion.
- Ordinary motion is intentionally silent. Persistent search/follower missions
  announce only meaningful phase changes rather than narrating every frame or
  small movement.

### Camera missions and local follower

Two related behaviors exist:

1. Persistent target mission: instructions such as "find a shoe and move towards
   it" use repeated fresh camera inspections plus short search, alignment, and
   approach actions until reached, explicitly stopped, timed out, or blocked.
2. Local follower mode: "follow me" or "follow this dog" uses one vision call to
   identify and seed a selected target, then an OpenCV tracker and LiDAR perform
   continuous local following without repeated LLM calls. Camera supplies target
   bearing; LiDAR supplies range and safe geometry. Losing the target stops the
   robot until it can be reacquired.

No two motion controllers should run simultaneously. A direct movement command or
new mission stops follower mode first.

### WebUI and live view

The Arduino WebUI brick serves the interface on port 7000. The Python console
prints the authoritative Local and Network URLs. Typical access is:

- `http://arduinoq.local:7000`
- or the printed IP URL, for example `http://<UNO-Q-IP>:7000`

The bottom-right globe icon in App Lab is not the camera viewer. Open the printed
URL in a browser. The page shows camera video, LiDAR points/sectors, navigation
events, supplemental sensor status, follower state, and an emergency-stop action.

## 4. Repository layout

| File | Purpose |
|---|---|
| `main.py` | Linux application lifecycle, audio queue, commands, missions, follower coordination |
| `python/main.py` | Required App Lab Python entry point; imports root `main.py` |
| `audio_io.py` | USB microphone selection/recovery, VAD, resampling, streamed playback |
| `robot_agent.py` | direct parser, tool schemas/validation, LLM, STT, vision, TTS, target parsing |
| `execute_motion_bridge.py` | validated Python-to-MCU RPC and motion polling |
| `lidar_x2.py` | custom X2 packet decoder, scan sectors, freshness, 10 cm guard |
| `navigation_controller.py` | deterministic local avoidance, corridor correction, recovery |
| `local_follower.py` | camera/LiDAR local target-following loop |
| `live_view.py` | WebUI publisher and emergency-stop handler |
| `assets/` | live camera/LiDAR browser interface |
| `sketch/sketch.ino` | MCU motors, ultrasonic, MPU, watchdog, Bridge services |
| `sketch/sketch.yaml` | UNO Q platform and exact Arduino library versions |
| `app.yaml` | App Lab metadata and WebUI brick |
| `.env.example` | safe configuration template; never place real keys here |
| `test_*.py` | desktop unit and packaging regression tests |

The import ZIP must have `app.yaml`, root `main.py`, `python/main.py`,
`python/requirements.txt`, `sketch/sketch.ino`, and `sketch/sketch.yaml` at exactly
those paths. A previous ZIP nested the project one directory too deep and App Lab
reported "main python file missing from app". The v18 ZIP was inspected and has
the correct flat structure.

## 5. Starting and testing v18.0

1. Copy `.env.example` to `.env` in the App Lab app and provide valid keys.
2. Import `uno-q-robot-app-v18.0-final-navigation.zip` into Arduino App Lab.
3. Open the imported app and press **Run** once. App Lab compiles/flashes the MCU
   and starts the Python container together.
4. Keep the robot raised or wheels unloaded for the first motor-direction test.
5. Confirm the log reports `navigation-v18.0`, MCU Bridge ready, a fresh LiDAR
   scan, selected EMEET microphone, camera availability, and the WebUI URL.
6. Put the robot on a clear controlled floor only after verifying `/status`,
   `/lidar`, `/sensors`, and `/test-camera`.

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

Desktop regression command:

```powershell
python -m unittest discover -v
```

At this handoff, all 97 desktop tests pass. This does not replace compiling the
sketch in App Lab or testing real wheel direction, sensors, and LiDAR on hardware.

## 6. Chronological development history and solved problems

### Initial sensor/motor test

The project began with four BO motors, two TB6612FNG drivers, two VL53L0X ToF
sensors, one ultrasonic sensor, and an MPU6050. The first sketch read all sensors
and looped forward/stop/backward motor tests. Dual ToF devices required XSHUT
sequencing and unique I2C addresses.

For historical reference only, the retired ToF wiring was:

| Retired device | Power/bus | XSHUT |
|---|---|---|
| Front-left VL53L0X | Buck #2 5 V, common GND, shared I2C SDA/SCL | D10 |
| Front-right VL53L0X | Buck #2 5 V, common GND, shared I2C SDA/SCL | D11 |

Both sensors initially power up at the same I2C address, so the code held both in
reset, enabled them one at a time, and assigned separate addresses. They were
later removed from both the final hardware architecture and v18 software. Do not
reconnect them unless the architecture is deliberately changed again.

### Arduino App Lab packaging and library failures

- App Lab imports ZIP archives, not an arbitrary individual source file.
- Both the sketch and Python app must be in one correctly structured archive.
- Read-only `sketch.yaml` in App Lab is generated/managed from the imported
  project, so dependencies had to be fixed in the source archive before import.
- Initial Adafruit library names were not resolvable. Exact names/versions were
  added, followed by the missing `Adafruit BusIO` transitive dependency.
- A C++ compile error occurred because `ProximitySample` was referenced before
  its definition. Its declaration order was corrected.
- A later OpenCV requirement used an unavailable exact build suffix and caused
  dependency resolution to fail, which also left `groq` unavailable. The current
  requirements do not reinstall App Lab's base OpenCV package.

### Voice pipeline and audio overflow

Early builds appeared stuck in `listening` because speech RMS never crossed the
threshold, and PortAudio repeatedly reported input overflow. Work included:

- listing visible input devices and sample rates
- selecting stable device names rather than volatile indices
- resampling device-native audio to 16 kHz for VAD/STT
- limiting and draining capture queues
- restarting the stream after repeated overflow instead of crashing the app
- capping calibration so a noisy calibration cannot make speech impossible
- preserving pre-roll and reliable end-of-speech detection
- pausing listening during playback/motion

The analog external-microphone attempt selected `USB PnP Sound Device`, but the
TRRS-to-TRS adapter path overflowed and sounded worse. v15.5/v15.6 changes were
reverted. EMEET camera audio is now preferred and USB loss triggers automatic
PortAudio refresh and reconnection rather than application shutdown.

### Physical motor control and mapping

The original agent only returned simulated JSON from `execute_motion`. A
RouterBridge sketch and Python backend were added so validated tool calls drive
real GPIO. The MCU motion is non-blocking and Python polls it so explicit stops
remain responsive.

Several rounds of wheel testing exposed wrong channel ownership, individual
polarity errors, a rear-left reverse issue, wrong rear-wheel tank-turn signs, and
finally semantic left/right inversion. These were resolved using the exact
physical driver-output table and per-wheel polarity constants now in v18. Forward,
backward, left, and right are covered by regression tests.

### Distance control replaced by time control

Commands such as "move 50 cm" were inaccurate because the motors have no
encoders. RPM-based estimates could not account for battery voltage, load, wheel
slip, or motor mismatch. The public contract was changed to seconds and speed
percentage. This is why the current parser rejects centimeters and degrees.

### Latency, speech chatter, and command confusion

- Direct deterministic motion parsing prevents chat history from turning
  "forward" into "left" or another stale action.
- Camera inspection became on-demand rather than happening for every utterance.
- Camera retry/backoff and re-enumeration prevent transient USB failures from
  killing the process.
- TTS became streaming and playback volume gained a configurable multiplier.
- Movement narration was reduced; local missions announce phase changes only.
- Longer missions gained persistent search/align/approach loops rather than one
  or two motor steps followed by an unexplained stop.

### Live camera and App Lab WebUI

The live view was implemented with the Arduino WebUI brick. Initial connection
refusals occurred because the Python application crashed during audio calibration;
when the app exits, port 7000 naturally disappears. Audio failure recovery now
keeps the process alive, and the browser should use the Network URL printed by
WebUI rather than the App Lab globe icon.

### LiDAR and evolving safety architecture

Some interim motor-only releases intentionally disabled sensor guards so wheel
mapping could be tested. Those are not the current design. YDLIDAR X2 support was
then added with live polar data, scan freshness, sector clearances, obstacle
avoidance, and an emergency boundary fixed at exactly 10 cm or less.

An intermediate v17 design included two ToF sensors. The final requested hardware
architecture removed both ToFs and all extra ultrasonics. v18 retains only:

- YDLIDAR X2 as primary navigation
- one front ultrasonic as independent emergency backup
- MPU6050 for heading/turn/tilt support
- camera for identification and high-level vision

### Packaging fixes

At least two archives failed import because the required main Python file was not
at the expected path. The stable solution is a root `main.py` plus the lightweight
`python/main.py` App Lab wrapper. Current ZIP contents were explicitly verified.

## 7. Known limitations of the current custom navigation

- There are no wheel encoders and therefore no measured wheel velocity or true
  metric wheel odometry.
- Timed PWM motion is not guaranteed to travel an exact distance.
- Current mapping/navigation is custom local sector logic, not ROS 2, Nav2, or
  SLAM Toolbox.
- MPU6050 yaw is relative and drifts; accelerometer double integration is not a
  usable replacement for translational odometry.
- The custom LiDAR reader must see fresh X2 packets. Required LiDAR fails closed
  for forward motion.
- Dynamic environments, glass, mirrors, cable clutter, wheel slip, and featureless
  walls can reduce navigation reliability.
- The 2 GB UNO Q is resource-constrained for simultaneous ROS, SLAM, Nav2, camera,
  OpenCV, App Lab, and voice services.

## 8. Last read-only UNO Q system inspection

The board was inspected through SSH only; nothing was installed or modified.

- Hostname: `ArduinoQ`
- User: `arduino`
- OS: Debian GNU/Linux 13.2 (trixie)
- Architecture: `aarch64`
- Kernel observed: Linux 6.16.7
- RAM visible: approximately 1.7 GiB, with about 1.0 GiB available at inspection
- Swap: approximately 870 MiB
- Root filesystem: 9.8 GB total, 8.2 GB used, approximately 1.1 GB free (89% used)
- Python: 3.13.5
- Docker: available and functioning, version 26.1.5, overlay2 storage
- ROS 2: not installed; no `ros2`, no `/opt/ros`, no installed ROS packages found
- USB at inspection: no Linux USB devices enumerated
- YDLIDAR at inspection: no `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/ydlidar`, or
  `/dev/serial/by-id`; therefore it was not detected at that moment

USB state can change after reconnecting hardware, so the next session should
re-check rather than assume the LiDAR is still absent.

## 9. Planned ROS 2 navigation direction

The intended future architecture is:

- ROS 2 Jazzy
- Nav2
- SLAM Toolbox
- official YDLIDAR ROS 2 driver publishing `/scan`
- UNO Q Linux side running the ROS navigation graph
- existing UNO Q MCU retaining hard real-time motor, watchdog, and emergency stop
- camera/LLM remaining separate and issuing only high-level goals
- local navigation never waiting for the LLM or TTS

ROS 2 Jazzy binary support targets Ubuntu 24.04 ARM64, whereas the UNO Q host is
Debian 13. The practical plan is therefore a minimal Ubuntu 24.04 ARM64 Docker
container using `ros:jazzy-ros-base`, not ROS Desktop and not a native Debian
binary installation. RViz should run on the Windows PC over the network.

### Running without encoder motors

Nav2 does not require wheel encoders, but it absolutely requires a smooth,
continuous local odometry source that publishes `nav_msgs/Odometry` and
`odom -> base_link`. SLAM Toolbox supplies global `map -> odom`; it does not
replace local odometry.

The recommended no-encoder prototype is:

```text
YDLIDAR /scan -> RF2O ROS 2 laser odometry ----+
MPU6050 /imu/data -----------------------------+-> robot_localization EKF
                                                    -> /odometry/filtered
                                                    -> odom -> base_link

SLAM Toolbox: /scan + odom TF -> /map + map -> odom
Nav2: map + scan + filtered odometry -> /cmd_vel
/cmd_vel -> velocity/PWM adapter -> existing Bridge -> MCU
```

RF2O is a candidate ROS 2 2D laser-odometry package designed for robots with poor
or absent wheel odometry. Its ROS 2 branch must be source-built and tested on
Jazzy/aarch64; compatibility must not be assumed. If an EKF publishes the final
TF, RF2O must not simultaneously publish a competing `odom -> base_link` TF.

This is viable for a slow indoor prototype, but less robust than encoder odometry.
Laser odometry may degrade during fast turns, in featureless corridors, around
moving people, with poor reflective surfaces, or when the LiDAR vibrates. The
MPU helps angular motion but cannot provide reliable X/Y translation alone.
Commanded-PWM/time odometry is only a temporary bootstrap and not an acceptable
primary source for autonomous navigation.

### External storage plan

Arduino officially supports external microSD or USB storage through a USB-C
dongle/hub with external power delivery. This is a mounted external filesystem;
it does not automatically enlarge `/`, and it does not increase RAM.

Recommended options:

1. 128 GB high-endurance A2/U3 microSD in a powered USB-C hub.
2. Preferably, a USB SSD on the powered hub for Docker/colcon performance and
   write endurance.
3. Use `ext4` and place Docker volumes/data, ROS workspaces, `build/install/log`,
   maps, and rosbag data on external storage.
4. Do not depend on microSD swap to solve the 2 GB RAM constraint.

Moving Docker's data root is a disruptive configuration change and has not been
performed. It should be planned only after the external device is detected and
its stable mount is verified.

### Required next phase, in order

No ROS changes have been made yet. The next chat should proceed incrementally:

1. Re-inspect USB and confirm powered-hub/microSD or SSD detection.
2. Confirm YDLIDAR appears on Linux and validate raw packets before ROS.
3. Decide external-storage mount and Docker-data strategy.
4. Start a minimal Ubuntu 24.04 ARM64 ROS Jazzy container.
5. Validate basic ROS networking from the Windows development PC.
6. Build the YDLIDAR SDK/ROS 2 driver and validate `/scan` plus TF.
7. Build and benchmark RF2O; validate stable laser odometry at slow speeds.
8. Publish MPU6050 IMU data and fuse it with laser odometry using
   `robot_localization`.
9. Run SLAM Toolbox and create/save a map.
10. Configure Nav2 against the saved map and connect `/cmd_vel` to a new Bridge
    base-driver node.
11. Preserve MCU ultrasonic, tilt, watchdog, short brake, and emergency stop as a
    final independent layer below ROS.
12. Only after navigation is stable, reconnect camera/LLM high-level goals and
    later rebuild human/animal following on top of Nav2.

Do not run the current `NavigationController` and Nav2 against the motors at the
same time. During migration, only one component may own motion commands.

## 10. Guardrails for the next chat

- Inspect first; do not install or modify the UNO Q unless the user explicitly
  authorizes that phase.
- Preserve the final motor channel map and polarity constants.
- Preserve A3=TRIG and A4=ECHO.
- Preserve physical SDA/SCL wording and MPU address `0x68`.
- Do not reintroduce ToF sensors or the external analog microphone.
- Do not include `.env`, passwords, or real API keys in ZIPs or commits.
- Do not claim centimeter-accurate motion without a measured odometry source.
- Do not let the LLM directly control raw LiDAR, SLAM, odometry, collision safety,
  or the real-time follower loop.
- Keep the MCU emergency and watchdog layer even after Nav2 is operational.
- Validate each layer independently before moving to the next one.

## 11. Primary technical references for the migration

- Arduino UNO Q hardware and external-storage support:
  <https://docs.arduino.cc/hardware/uno-q>
- ROS 2 Jazzy Ubuntu 24.04 ARM64 support:
  <https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html>
- Nav2 state-estimation and transform requirements:
  <https://docs.nav2.org/concepts/>
- Nav2 odometry setup (encoders are not mandatory):
  <https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html>
- `robot_localization` with odometry and IMU:
  <https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html>
- Official YDLIDAR ROS 2 driver:
  <https://github.com/YDLIDAR/ydlidar_ros2_driver>
- RF2O ROS 2 laser-odometry branch:
  <https://github.com/MAPIRlab/rf2o_laser_odometry/tree/ros2>
