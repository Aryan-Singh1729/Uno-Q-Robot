# WALL-E UNO Q final navigation — v18.0

This Arduino App Lab application runs the STM32 motor/safety sketch and the Linux
Python voice, camera, LiDAR, navigation, and WebUI services together.

## Final architecture

```text
Camera identification ──> local visual tracker ─┐
                                                ├─> NavigationController ─> Bridge ─> motors
YDLIDAR X2 scan ────────────────────────────────┘             ^
                                                              │
Front ultrasonic emergency stop + MPU6050 correction ─────────┘

Voice ─> STT ─> high-level command/LLM
                    │
                    └─> start/stop follower or request a motion
```

LiDAR is the primary navigation sensor. It detects walls and obstacles, measures
free space, centers the robot in a corridor, chooses local recovery directions, and
guards every motor primitive. The front ultrasonic is only an independent direct-ahead
emergency backup. The MPU6050 provides tilt stopping, measured turn progress, straight-line
heading correction, and drift events. These loops run locally and never wait for an LLM,
network response, or speech playback.

Follower mode uses exactly one vision request to identify a specific visible person or
animal and seed a local OpenCV tracker. After that, the camera tracker supplies bearing,
LiDAR supplies range and surrounding geometry, and the local controller continuously issues
short safe motions. It does not make repeated LLM calls. If the selected target is lost, the
robot stops moving and waits for the same local appearance to become visible again.

## Final wiring

All grounds must be common.

| Device | Signal | UNO Q connection |
|---|---|---|
| Front ultrasonic | VCC / GND | Buck #2 5 V / common GND |
| Front ultrasonic | TRIG / ECHO | A3 / A4 (preserved tested mapping) |
| MPU6050 | VCC / GND | Buck #2 5 V / common GND |
| MPU6050 | SDA / SCL | Physical **SDA** / physical **SCL** header pins |
| MPU6050 | XDA / XCL / INT | Not connected |
| MPU6050 | AD0 | Default, giving address `0x68` |
| YDLIDAR X2 | data | Existing Linux USB serial adapter |

The sketch calls `Wire.begin()` so it uses the UNO Q's dedicated physical SDA/SCL
header pins. It does not remap or describe this bus using digital-pin aliases.

Motor wiring is fixed as follows:

| Driver/channel | Motor | Control pins |
|---|---|---|
| TB6612 #1 A | Front left | PWMA D3, AIN1 D4, AIN2 D7 |
| TB6612 #1 B | Rear right | PWMB D5, BIN1 D8, BIN2 D12 |
| TB6612 #2 A | Front right | PWMA D6, AIN1 D13, AIN2 A0 |
| TB6612 #2 B | Rear left | PWMB D9, BIN1 A1, BIN2 A2 |
| Both drivers | STBY | D2 shared |

Both TB6612 boards use Buck #2 5 V for VCC and VM and common GND. The GPIO numbers,
electrical polarity constants, Bridge watchdog, short brake, and stop RPC remain in the
STM32 sketch.

## Start in Arduino App Lab

1. Copy `.env.example` to `.env` and set your API keys. Never share the real `.env`.
2. Import `uno-q-robot-app-v18.0-final-navigation.zip` in Arduino App Lab.
3. Open the imported app and press **Run**. Do not run `main.py` separately over SSH.
4. Confirm the Python console shows a fresh YDLIDAR scan and the MCU reports
   `navigation-v18.0`.
5. Open the Network URL printed by WebUI, normally `http://arduinoq.local:7000`, for
   camera, LiDAR, sensor, navigation-event, and follower status.

Useful console commands:

```text
/status
/lidar
/sensors
/test-camera
/follow person
/follow dog
/unfollow
/stop
```

Voice examples include “follow me”, “follow this dog”, and “stop following”. A direct
movement command or a new target mission automatically stops follower mode so two controllers
cannot compete for the motors.

## Safety behavior

- LiDAR is required for physical forward navigation. Missing or stale scans fail closed.
- LiDAR's hard emergency boundary remains exactly 100 mm (10 cm) or less.
- A valid front ultrasonic reading at 100 mm or less immediately stops forward motion on the
  MCU. It is not used for route planning or follower distance.
- Excessive MPU tilt stops motion immediately. MPU yaw feedback corrects straight movement and
  detects a commanded turn that produced no rotation.
- The MCU controller watchdog and WebUI/console emergency stop remain active.

Run the desktop test suite with `python -m unittest discover -v`. The real sketch must still
be compiled and flashed by Arduino App Lab because the UNO Q Zephyr core and RouterBridge
libraries are managed there.
