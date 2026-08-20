/*
 * WALL-E UNO Q v18.0 - real-time motor and supplemental-sensor controller
 *
 * Linux remains responsible for the primary YDLIDAR X2 navigation scan. This
 * MCU layer independently enforces the front ultrasonic, MPU6050 tilt,
 * motion watchdog, and short-brake safety paths. Sensor stops do
 * not wait for Python, the LLM, networking, or speech.
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static const float DEG_PER_SEC_AT_100 = 180.0f;
static const float MAX_DRIVE_SECONDS = 120.0f;
static const float MAX_COMMAND_ANGLE_DEG = 3600.0f;
static const float MAX_SPIN_SECONDS = 120.0f;
static const int MAX_COMMAND_SPEED = 100;
static const uint32_t MIN_MOTION_DURATION_MS = 1;
static const uint32_t MAX_MOTION_DURATION_MS = 600000;
static const uint32_t SHORT_BRAKE_MS = 100;
static const uint32_t CONTROLLER_WATCHDOG_MS = 750;

static const uint16_t CLOSE_STOP_MM = 100;  // exactly 10 cm or less
static const float MAX_TILT_DEG = 28.0f;
static const float HEADING_EVENT_DEG = 3.0f;
static const float HEADING_KP_PWM_PER_DEG = 2.0f;
static const int MAX_HEADING_PWM_CORRECTION = 24;
static const uint32_t ULTRASONIC_PERIOD_MS = 55;
static const uint32_t IMU_PERIOD_MS = 20;
static const uint32_t SENSOR_STALE_MS = 500;
static const uint32_t EVENT_RATE_LIMIT_MS = 750;
static const uint32_t TURN_STUCK_CHECK_MS = 800;
static const float MIN_TURN_PROGRESS_DEG = 2.0f;

// Both TB6612FNG STBY pins are tied to D2.
static const uint8_t STBY_PIN = 2;

// Final physical channel ownership. Do not reorder these channels.
// Driver 1: channel A front-left, channel B rear-right.
static const uint8_t D1_PWMA = 3;
static const uint8_t D1_AIN1 = 4;
static const uint8_t D1_AIN2 = 7;
static const uint8_t D1_PWMB = 5;
static const uint8_t D1_BIN1 = 8;
static const uint8_t D1_BIN2 = 12;

// Driver 2: channel A front-right, channel B rear-left.
static const uint8_t D2_PWMA = 6;
static const uint8_t D2_AIN1 = 13;
static const uint8_t D2_AIN2 = A0;
static const uint8_t D2_PWMB = 9;
static const uint8_t D2_BIN1 = A1;
static const uint8_t D2_BIN2 = A2;

// Recovered tested sensor mapping. The historical connection table listed
// TRIG/ECHO in this A3/A4 order; keep these named constants together.
static const uint8_t ULTRASONIC_TRIG_PIN = A3;
static const uint8_t ULTRASONIC_ECHO_PIN = A4;

// MPU6050 uses the UNO Q's physical SDA/SCL header pins through Wire. AD0 is
// left at its default address (0x68); XDA, XCL and INT are not connected.
static const uint8_t MPU_SDA_PIN = SDA;
static const uint8_t MPU_SCL_PIN = SCL;

// Logical-forward polarities from the completed four-wheel floor tests.
static const int FRONT_LEFT_POLARITY = -1;
static const int FRONT_RIGHT_POLARITY = 1;
static const int REAR_LEFT_POLARITY = -1;
static const int REAR_RIGHT_POLARITY = 1;

enum MotionKind : uint8_t { MOTION_NONE, MOTION_DRIVE, MOTION_TURN };

Adafruit_MPU6050 mpu;

uint32_t motionId = 0;
uint32_t motionDeadline = 0;
uint32_t brakeReleaseDeadline = 0;
uint32_t lastControllerContact = 0;
uint32_t motionStartedAt = 0;
bool bridgeReady = false;
bool motorsRunning = false;
bool shortBrakeActive = false;
String motionStatus = "idle";
String motionReason = "startup";
MotionKind motionKind = MOTION_NONE;
int commandDirection = 0;
uint8_t commandPwm = 0;

bool mpuReady = false;
bool ultrasonicReady = false;
uint16_t ultrasonicMm = 0;
uint32_t ultrasonicAt = 0;
uint32_t imuAt = 0;
uint32_t lastUltrasonicPoll = 0;
uint32_t lastImuPoll = 0;
float rollDeg = 0.0f;
float pitchDeg = 0.0f;
float yawDeg = 0.0f;
float motionStartYawDeg = 0.0f;
float gyroBiasZ = 0.0f;
float rollReferenceDeg = 0.0f;
float pitchReferenceDeg = 0.0f;
uint8_t tiltHitCount = 0;

uint32_t navigationEventSequence = 0;
uint32_t lastEventAt = 0;
String navigationEvent = "";
String navigationEventDetail = "";

static bool timeReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static bool freshReading(uint32_t timestamp, uint32_t now) {
  return timestamp != 0 && (uint32_t)(now - timestamp) <= SENSOR_STALE_MS;
}

static uint8_t speedToPWM(int speed) {
  return (uint8_t)map(constrain(speed, 0, 100), 0, 100, 0, 255);
}

static void setMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, int direction, uint8_t pwm) {
  if (direction > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (direction < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwmPin, direction == 0 ? 0 : pwm);
}

static void setDriveMotors(int direction, uint8_t leftPwm, uint8_t rightPwm) {
  setMotor(D1_AIN1, D1_AIN2, D1_PWMA, direction * FRONT_LEFT_POLARITY, leftPwm);
  setMotor(D1_BIN1, D1_BIN2, D1_PWMB, direction * REAR_RIGHT_POLARITY, rightPwm);
  setMotor(D2_AIN1, D2_AIN2, D2_PWMA, direction * FRONT_RIGHT_POLARITY, rightPwm);
  setMotor(D2_BIN1, D2_BIN2, D2_PWMB, direction * REAR_LEFT_POLARITY, leftPwm);
}

static void setAllMotors(int direction, uint8_t pwm) {
  setDriveMotors(direction, pwm, pwm);
}

static void setTankTurn(int direction, uint8_t pwm) {
  // Positive is semantic left: left wheels reverse and right wheels advance.
  const int left = direction > 0 ? -1 : 1;
  const int right = direction > 0 ? 1 : -1;
  setMotor(D1_AIN1, D1_AIN2, D1_PWMA, left * FRONT_LEFT_POLARITY, pwm);
  setMotor(D1_BIN1, D1_BIN2, D1_PWMB, right * REAR_RIGHT_POLARITY, pwm);
  setMotor(D2_AIN1, D2_AIN2, D2_PWMA, right * FRONT_RIGHT_POLARITY, pwm);
  setMotor(D2_BIN1, D2_BIN2, D2_PWMB, left * REAR_LEFT_POLARITY, pwm);
}

static void releaseMotorOutputs() {
  setAllMotors(0, 0);
  digitalWrite(STBY_PIN, LOW);
  shortBrakeActive = false;
  motionKind = MOTION_NONE;
  commandDirection = 0;
  commandPwm = 0;
}

static void engageShortBrake() {
  digitalWrite(STBY_PIN, HIGH);
  const uint8_t directionPins[] = {
    D1_AIN1, D1_AIN2, D1_BIN1, D1_BIN2,
    D2_AIN1, D2_AIN2, D2_BIN1, D2_BIN2
  };
  for (uint8_t pin : directionPins) digitalWrite(pin, HIGH);
  analogWrite(D1_PWMA, 255);
  analogWrite(D1_PWMB, 255);
  analogWrite(D2_PWMA, 255);
  analogWrite(D2_PWMB, 255);
  shortBrakeActive = true;
  brakeReleaseDeadline = millis() + SHORT_BRAKE_MS;
}

static void publishNavigationEvent(const String &event, const String &detail, bool force = false) {
  const uint32_t now = millis();
  if (!force && event == navigationEvent && (uint32_t)(now - lastEventAt) < EVENT_RATE_LIMIT_MS) return;
  navigationEvent = event;
  navigationEventDetail = detail;
  navigationEventSequence++;
  lastEventAt = now;
  Serial.println("[NAV] " + event + ": " + detail);
}

static void finishMotion(const String &status, const String &reason) {
  if (motorsRunning || shortBrakeActive) engageShortBrake();
  motorsRunning = false;
  motionDeadline = 0;
  motionStatus = status;
  motionReason = reason;
  motionKind = MOTION_NONE;
  commandDirection = 0;
  commandPwm = 0;
}

static String jsonEscape(String value) {
  value.replace("\\", "\\\\");
  value.replace("\"", "\\\"");
  return value;
}

static String boolJson(bool value) {
  return value ? "true" : "false";
}

static String statusJson() {
  String json = "{";
  json += "\"ready\":" + boolJson(bridgeReady) + ",";
  json += "\"motion_id\":" + String(motionId) + ",";
  json += "\"status\":\"" + jsonEscape(motionStatus) + "\",";
  json += "\"reason\":\"" + jsonEscape(motionReason) + "\",";
  json += "\"firmware_version\":\"navigation-v18.0\",";
  json += "\"motor_map\":\"D1A=front_left,D1B=rear_right,D2A=front_right,D2B=rear_left\",";
  json += "\"sensor_guard_enabled\":true,";
  json += "\"lidar_guard_source\":\"linux_usb\",";
  json += "\"lidar_stop_distance_mm\":100,";
  json += "\"supplemental_stop_distance_mm\":" + String(CLOSE_STOP_MM) + ",";
  json += "\"navigation_event\":\"" + jsonEscape(navigationEvent) + "\",";
  json += "\"navigation_event_detail\":\"" + jsonEscape(navigationEventDetail) + "\",";
  json += "\"navigation_event_sequence\":" + String(navigationEventSequence);
  json += "}";
  return json;
}

static String sensorJson() {
  const uint32_t now = millis();
  String json = "{";
  json += "\"status\":\"ok\",";
  json += "\"ultrasonic_ready\":" + boolJson(ultrasonicReady && freshReading(ultrasonicAt, now)) + ",";
  json += "\"mpu6050_ready\":" + boolJson(mpuReady) + ",";
  json += "\"ultrasonic_mm\":" + String(freshReading(ultrasonicAt, now) ? ultrasonicMm : 0) + ",";
  json += "\"roll_deg\":" + String(rollDeg, 2) + ",";
  json += "\"pitch_deg\":" + String(pitchDeg, 2) + ",";
  json += "\"yaw_deg\":" + String(yawDeg, 2) + ",";
  json += "\"navigation_event\":\"" + jsonEscape(navigationEvent) + "\",";
  json += "\"navigation_event_detail\":\"" + jsonEscape(navigationEventDetail) + "\",";
  json += "\"navigation_event_sequence\":" + String(navigationEventSequence);
  json += "}";
  return json;
}

static void calibrateGyroBias() {
  if (!mpuReady) return;
  float sum = 0.0f;
  float rollSum = 0.0f;
  float pitchSum = 0.0f;
  const int samples = 80;
  sensors_event_t accel, gyro, temp;
  for (int i = 0; i < samples; ++i) {
    mpu.getEvent(&accel, &gyro, &temp);
    sum += gyro.gyro.z;
    rollSum += atan2f(accel.acceleration.y, accel.acceleration.z) * 180.0f / PI;
    pitchSum += atan2f(
      -accel.acceleration.x,
      sqrtf(accel.acceleration.y * accel.acceleration.y +
             accel.acceleration.z * accel.acceleration.z)
    ) * 180.0f / PI;
    delay(5);
  }
  gyroBiasZ = sum / samples;
  rollReferenceDeg = rollSum / samples;
  pitchReferenceDeg = pitchSum / samples;
}

static void initializeSensors() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  (void)MPU_SDA_PIN;
  (void)MPU_SCL_PIN;
  Wire.begin();
  Wire.setClock(400000);
  mpuReady = mpu.begin(0x68, &Wire);
  if (mpuReady) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    calibrateGyroBias();
  } else {
    Serial.println("[SENSOR] MPU6050 unavailable at 0x68");
  }
}

static void sampleUltrasonic(uint32_t now) {
  if ((uint32_t)(now - lastUltrasonicPoll) < ULTRASONIC_PERIOD_MS) return;
  lastUltrasonicPoll = now;
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  const unsigned long echoUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 7000UL);
  if (echoUs > 0) {
    ultrasonicMm = (uint16_t)min(2000UL, (echoUs * 10UL) / 58UL);
    ultrasonicAt = now;
    ultrasonicReady = true;
  }
}

static void sampleImu(uint32_t now) {
  if (!mpuReady || (uint32_t)(now - lastImuPoll) < IMU_PERIOD_MS) return;
  const float dt = lastImuPoll == 0 ? 0.0f : (now - lastImuPoll) / 1000.0f;
  lastImuPoll = now;
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);
  imuAt = now;
  const float ax = accel.acceleration.x;
  const float ay = accel.acceleration.y;
  const float az = accel.acceleration.z;
  rollDeg = atan2f(ay, az) * 180.0f / PI - rollReferenceDeg;
  pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI - pitchReferenceDeg;
  yawDeg += (gyro.gyro.z - gyroBiasZ) * dt * 180.0f / PI;
}

static void applyHeadingCorrection() {
  if (!motorsRunning || motionKind != MOTION_DRIVE || !mpuReady) return;
  const float error = yawDeg - motionStartYawDeg;
  int correction = (int)roundf(error * HEADING_KP_PWM_PER_DEG);
  correction = constrain(correction, -MAX_HEADING_PWM_CORRECTION, MAX_HEADING_PWM_CORRECTION);
  if (commandDirection < 0) correction = -correction;
  const uint8_t leftPwm = (uint8_t)constrain((int)commandPwm + correction, 0, 255);
  const uint8_t rightPwm = (uint8_t)constrain((int)commandPwm - correction, 0, 255);
  setDriveMotors(commandDirection, leftPwm, rightPwm);
  if (fabs(error) >= HEADING_EVENT_DEG) {
    publishNavigationEvent("HEADING_CORRECTION", "correcting " + String(error, 1) + " degree drift");
  }
}

static void detectTurnStall(uint32_t now) {
  if (!motorsRunning || motionKind != MOTION_TURN || !mpuReady ||
      (uint32_t)(now - motionStartedAt) < TURN_STUCK_CHECK_MS) return;
  const float progress = fabs(yawDeg - motionStartYawDeg);
  if (progress >= MIN_TURN_PROGRESS_DEG) return;
  publishNavigationEvent("ROBOT_STUCK", "turn command produced no measured rotation", true);
  finishMotion("obstacle", "MPU6050 detected stalled turn");
}

static void enforceSupplementalSafety(uint32_t now) {
  if (!motorsRunning) {
    tiltHitCount = 0;
    return;
  }
  const bool tilted = mpuReady && freshReading(imuAt, now) &&
    (fabs(rollDeg) >= MAX_TILT_DEG || fabs(pitchDeg) >= MAX_TILT_DEG);
  tiltHitCount = tilted ? min((uint8_t)3, (uint8_t)(tiltHitCount + 1)) : 0;
  if (tiltHitCount >= 2) {
    const String detail = "roll=" + String(rollDeg, 1) + ", pitch=" + String(pitchDeg, 1);
    publishNavigationEvent("TILT_WARNING", detail, true);
    finishMotion("obstacle", "MPU6050 excessive tilt: " + detail);
    return;
  }

  const bool centerClose = ultrasonicReady && freshReading(ultrasonicAt, now) &&
    ultrasonicMm > 0 && ultrasonicMm <= CLOSE_STOP_MM;
  // This one sensor is a direct-ahead emergency backup, not a navigation input.
  // A dangerously close reading stops every active motion immediately.
  const bool blocked = centerClose;
  if (!blocked) return;

  const String detail = "front ultrasonic at " + String(ultrasonicMm) + " mm";
  publishNavigationEvent("OBSTACLE_DETECTED", detail, true);
  finishMotion("obstacle", detail);
}

static void serviceSensorsAndNavigation() {
  const uint32_t now = millis();
  sampleUltrasonic(now);
  sampleImu(now);
  enforceSupplementalSafety(now);
  detectTurnStall(now);
  applyHeadingCorrection();
}

static String startMotion(int speed, float amount, bool turning) {
  const float maximum = turning ? MAX_COMMAND_ANGLE_DEG : MAX_DRIVE_SECONDS;
  if (speed < 1 || speed > MAX_COMMAND_SPEED || !isfinite(amount) ||
      amount == 0.0f || fabs(amount) > maximum) {
    motionStatus = "error";
    motionReason = turning ? "invalid turn command" : "invalid timed drive command";
    return statusJson();
  }
  if (motorsRunning || shortBrakeActive) {
    finishMotion("cancelled", "superseded");
    releaseMotorOutputs();
  }

  uint32_t durationMs;
  if (turning) {
    const float rateAtSpeed = DEG_PER_SEC_AT_100 * ((float)speed / 100.0f);
    durationMs = (uint32_t)((fabs(amount) / rateAtSpeed) * 1000.0f);
  } else {
    durationMs = (uint32_t)(fabs(amount) * 1000.0f);
  }
  durationMs = max(MIN_MOTION_DURATION_MS, durationMs);
  if (durationMs > MAX_MOTION_DURATION_MS) {
    motionStatus = "error";
    motionReason = "requested motion exceeds controller duration";
    return statusJson();
  }

  ++motionId;
  commandDirection = amount > 0 ? 1 : -1;
  commandPwm = speedToPWM(speed);
  motionKind = turning ? MOTION_TURN : MOTION_DRIVE;
  motionStartYawDeg = yawDeg;
  motionStartedAt = millis();
  digitalWrite(STBY_PIN, HIGH);
  if (turning) setTankTurn(commandDirection, commandPwm);
  else setAllMotors(commandDirection, commandPwm);
  motorsRunning = true;
  shortBrakeActive = false;
  motionStatus = "running";
  motionReason = "";
  motionDeadline = millis() + durationMs;
  lastControllerContact = millis();
  String json = statusJson();
  json.remove(json.length() - 1);
  json += ",\"duration_ms\":" + String(durationMs) + "}";
  return json;
}

String move_robot(int speed, float seconds) {
  return startMotion(speed, seconds, false);
}

String turn_robot(int speed, float angleDegrees) {
  return startMotion(speed, angleDegrees, true);
}

String spin_robot(int speed, float seconds) {
  if (speed < 1 || speed > MAX_COMMAND_SPEED || !isfinite(seconds) ||
      seconds == 0.0f || fabs(seconds) > MAX_SPIN_SECONDS) {
    motionStatus = "error";
    motionReason = "invalid timed spin command";
    return statusJson();
  }
  if (motorsRunning || shortBrakeActive) {
    finishMotion("cancelled", "superseded");
    releaseMotorOutputs();
  }
  uint32_t durationMs = max(MIN_MOTION_DURATION_MS, (uint32_t)(fabs(seconds) * 1000.0f));
  ++motionId;
  commandDirection = seconds > 0 ? 1 : -1;
  commandPwm = speedToPWM(speed);
  motionKind = MOTION_TURN;
  motionStartYawDeg = yawDeg;
  motionStartedAt = millis();
  digitalWrite(STBY_PIN, HIGH);
  setTankTurn(commandDirection, commandPwm);
  motorsRunning = true;
  shortBrakeActive = false;
  motionStatus = "running";
  motionReason = "";
  motionDeadline = millis() + durationMs;
  lastControllerContact = millis();
  String json = statusJson();
  json.remove(json.length() - 1);
  json += ",\"duration_ms\":" + String(durationMs) + "}";
  return json;
}

String stop_robot(String reason) {
  finishMotion("cancelled", reason.length() ? reason : "stop requested");
  return statusJson();
}

String robot_status() {
  lastControllerContact = millis();
  return statusJson();
}

String read_sensors() {
  return sensorJson();
}

void setup() {
  Serial.begin(115200);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);
  const uint8_t motorPins[] = {
    D1_PWMA, D1_AIN1, D1_AIN2, D1_PWMB, D1_BIN1, D1_BIN2,
    D2_PWMA, D2_AIN1, D2_AIN2, D2_PWMB, D2_BIN1, D2_BIN2
  };
  for (uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  initializeSensors();
  if (!Bridge.begin()) {
    motionStatus = "error";
    motionReason = "Bridge.begin failed";
    Serial.println("[MCU] Bridge.begin failed; motors disabled");
    return;
  }
  Bridge.provide_safe("move_robot", move_robot);
  Bridge.provide_safe("turn_robot", turn_robot);
  Bridge.provide_safe("spin_robot", spin_robot);
  Bridge.provide_safe("stop_robot", stop_robot);
  Bridge.provide_safe("robot_status", robot_status);
  Bridge.provide_safe("read_sensors", read_sensors);
  bridgeReady = true;
  motionReason = "navigation-v18.0 ready";
  publishNavigationEvent("NAVIGATION_READY", "LiDAR primary; ultrasonic emergency backup and MPU support", true);
  Serial.println("[MCU] navigation-v18.0 Bridge ready; Serial1 reserved for RouterBridge");
}

void loop() {
  serviceSensorsAndNavigation();
  const uint32_t now = millis();
  if (motorsRunning && timeReached(now, motionDeadline)) {
    finishMotion("completed", "duration reached");
    Serial.println("[MCU] motion completed");
  }
  if (motorsRunning && (uint32_t)(now - lastControllerContact) > CONTROLLER_WATCHDOG_MS) {
    finishMotion("cancelled", "Python controller watchdog expired");
    Serial.println("[MCU] controller watchdog stop");
  }
  if (shortBrakeActive && timeReached(now, brakeReleaseDeadline)) releaseMotorOutputs();
}
