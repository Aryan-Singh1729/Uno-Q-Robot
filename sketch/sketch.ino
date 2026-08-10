/*
 * UNO Q robot - motor-only App Lab test sketch
 *
 * Sensor integration is intentionally disabled in this build. The Linux
 * Python app calls these functions through Arduino_RouterBridge. Keep the
 * robot on a clear test surface and begin with short commands.
 */

#include <Arduino_RouterBridge.h>

static const float CM_PER_SEC_AT_100 = 30.0f;
static const float DEG_PER_SEC_AT_100 = 180.0f;
static const float MAX_COMMAND_DISTANCE_CM = 1000.0f;
static const float MAX_COMMAND_ANGLE_DEG = 3600.0f;
static const float MAX_SPIN_SECONDS = 120.0f;
static const int MAX_COMMAND_SPEED = 100;
static const uint32_t MIN_MOTION_DURATION_MS = 250;
static const uint32_t MAX_MOTION_DURATION_MS = 600000;
static const uint32_t SHORT_BRAKE_MS = 100;
static const uint32_t CONTROLLER_WATCHDOG_MS = 750;

// Both TB6612FNG STBY pins are tied to D2.
static const uint8_t STBY_PIN = 2;

// Driver 1: channel A front-left, channel B front-right.
static const uint8_t D1_PWMA = 3;
static const uint8_t D1_AIN1 = 4;
static const uint8_t D1_AIN2 = 7;
static const uint8_t D1_PWMB = 5;
static const uint8_t D1_BIN1 = 8;
static const uint8_t D1_BIN2 = 12;

// Driver 2: channel A rear-left, channel B rear-right.
static const uint8_t D2_PWMA = 6;
static const uint8_t D2_AIN1 = 13;
static const uint8_t D2_AIN2 = A0;
static const uint8_t D2_PWMB = 9;
static const uint8_t D2_BIN1 = A1;
static const uint8_t D2_BIN2 = A2;

// Change an individual value to -1 if that wheel spins opposite to the
// commanded direction. Do not compensate by changing the turn logic.
static const int FRONT_LEFT_POLARITY = 1;
static const int FRONT_RIGHT_POLARITY = 1;
static const int REAR_LEFT_POLARITY = 1;
static const int REAR_RIGHT_POLARITY = 1;

uint32_t motionId = 0;
uint32_t motionDeadline = 0;
uint32_t brakeReleaseDeadline = 0;
uint32_t lastControllerContact = 0;
bool bridgeReady = false;
bool motorsRunning = false;
bool shortBrakeActive = false;
String motionStatus = "idle";
String motionReason = "startup";

static bool timeReached(uint32_t now, uint32_t deadline) {
  return (int32_t)(now - deadline) >= 0;
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

static void setAllMotors(int direction, uint8_t pwm) {
  setMotor(D1_AIN1, D1_AIN2, D1_PWMA, direction * FRONT_LEFT_POLARITY, pwm);
  setMotor(D1_BIN1, D1_BIN2, D1_PWMB, direction * FRONT_RIGHT_POLARITY, pwm);
  setMotor(D2_AIN1, D2_AIN2, D2_PWMA, direction * REAR_LEFT_POLARITY, pwm);
  setMotor(D2_BIN1, D2_BIN2, D2_PWMB, direction * REAR_RIGHT_POLARITY, pwm);
}

static void setTankTurn(int direction, uint8_t pwm) {
  const int left = direction > 0 ? -1 : 1;
  const int right = direction > 0 ? 1 : -1;
  setMotor(D1_AIN1, D1_AIN2, D1_PWMA, left * FRONT_LEFT_POLARITY, pwm);
  setMotor(D2_AIN1, D2_AIN2, D2_PWMA, left * REAR_LEFT_POLARITY, pwm);
  setMotor(D1_BIN1, D1_BIN2, D1_PWMB, right * FRONT_RIGHT_POLARITY, pwm);
  setMotor(D2_BIN1, D2_BIN2, D2_PWMB, right * REAR_RIGHT_POLARITY, pwm);
}

static void releaseMotorOutputs() {
  setAllMotors(0, 0);
  digitalWrite(STBY_PIN, LOW);
  shortBrakeActive = false;
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

static void finishMotion(const String &status, const String &reason) {
  if (motorsRunning || shortBrakeActive) engageShortBrake();
  motorsRunning = false;
  motionDeadline = 0;
  motionStatus = status;
  motionReason = reason;
}

static String jsonEscape(String value) {
  value.replace("\\", "\\\\");
  value.replace("\"", "\\\"");
  return value;
}

static String statusJson() {
  String json = "{";
  json += "\"ready\":" + String(bridgeReady ? "true" : "false") + ",";
  json += "\"motion_id\":" + String(motionId) + ",";
  json += "\"status\":\"" + jsonEscape(motionStatus) + "\",";
  json += "\"reason\":\"" + jsonEscape(motionReason) + "\",";
  json += "\"firmware_version\":\"autonomy-v13\",";
  json += "\"motor_test_mode\":true,";
  json += "\"sensor_guard_enabled\":false";
  json += "}";
  return json;
}

static String startMotion(int speed, float amount, bool turning) {
  const float maximum = turning ? MAX_COMMAND_ANGLE_DEG : MAX_COMMAND_DISTANCE_CM;
  if (speed < 1 || speed > MAX_COMMAND_SPEED || !isfinite(amount) ||
      amount == 0.0f || fabs(amount) > maximum) {
    motionStatus = "error";
    motionReason = turning
      ? "invalid turn command"
      : "invalid move command";
    return statusJson();
  }

  if (motorsRunning || shortBrakeActive) {
    finishMotion("cancelled", "superseded");
    releaseMotorOutputs();
  }

  const float rateAtSpeed = (turning ? DEG_PER_SEC_AT_100 : CM_PER_SEC_AT_100)
                          * ((float)speed / 100.0f);
  uint32_t durationMs = (uint32_t)((fabs(amount) / rateAtSpeed) * 1000.0f);
  durationMs = max(MIN_MOTION_DURATION_MS, durationMs);
  if (durationMs > MAX_MOTION_DURATION_MS) {
    motionStatus = "error";
    motionReason = "requested motion exceeds controller duration";
    return statusJson();
  }

  ++motionId;
  const int direction = amount > 0 ? 1 : -1;
  digitalWrite(STBY_PIN, HIGH);
  if (turning) setTankTurn(direction, speedToPWM(speed));
  else setAllMotors(direction, speedToPWM(speed));

  motorsRunning = true;
  shortBrakeActive = false;
  motionStatus = "running";
  motionReason = "";
  motionDeadline = millis() + durationMs;
  lastControllerContact = millis();

  String json = "{";
  json += "\"motion_id\":" + String(motionId) + ",";
  json += "\"status\":\"running\",";
  json += "\"duration_ms\":" + String(durationMs) + ",";
  json += "\"motor_test_mode\":true,";
  json += "\"sensor_guard_enabled\":false";
  json += "}";
  return json;
}

String move_robot(int speed, float distanceCm) {
  return startMotion(speed, distanceCm, false);
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

  uint32_t durationMs = (uint32_t)(fabs(seconds) * 1000.0f);
  durationMs = max(MIN_MOTION_DURATION_MS, durationMs);
  ++motionId;
  digitalWrite(STBY_PIN, HIGH);
  setTankTurn(seconds > 0 ? 1 : -1, speedToPWM(speed));
  motorsRunning = true;
  shortBrakeActive = false;
  motionStatus = "running";
  motionReason = "";
  motionDeadline = millis() + durationMs;
  lastControllerContact = millis();

  String json = "{";
  json += "\"motion_id\":" + String(motionId) + ",";
  json += "\"status\":\"running\",";
  json += "\"duration_ms\":" + String(durationMs) + ",";
  json += "\"motor_test_mode\":true,";
  json += "\"sensor_guard_enabled\":false";
  json += "}";
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
  return "{\"status\":\"disabled\",\"reason\":\"motor-only test mode\"}";
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
  motionReason = "autonomy-v13 ready";
  Serial.println("[MCU] autonomy-v13 Bridge ready; sensor and obstacle guards disabled");
}

void loop() {
  const uint32_t now = millis();
  if (motorsRunning && timeReached(now, motionDeadline)) {
    finishMotion("completed", "duration reached");
    Serial.println("[MCU] motion completed");
  }
  if (motorsRunning && (uint32_t)(now - lastControllerContact) > CONTROLLER_WATCHDOG_MS) {
    finishMotion("cancelled", "Python controller watchdog expired");
    Serial.println("[MCU] controller watchdog stop");
  }
  if (shortBrakeActive && timeReached(now, brakeReleaseDeadline)) {
    releaseMotorOutputs();
  }
}
