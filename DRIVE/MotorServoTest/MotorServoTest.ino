#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// =====================================================
// WiFi config
// =====================================================
const char* WIFI_SSID = "ATTtzWcVAs";
const char* WIFI_PASS = "t8vzgkf3vfrb";

IPAddress local_IP(192, 168, 1, 247);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(8, 8, 8, 8);
IPAddress dns2(1, 1, 1, 1);

// =====================================================
// Web server
// =====================================================
WebServer server(80);

// =====================================================
// Servo
// =====================================================
static const int SERVO_PIN = 5;
Servo servo;
int servoAngle = 90;

// =====================================================
// Motor + Encoder
// =====================================================
#define RPWM_PIN 25
#define LPWM_PIN 26

#define ENC_A_PIN 34
#define ENC_B_PIN 35

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define PWM_MAX         255

static const float ENCODER_CPR = 64.0;
static const float GEAR_RATIO  = 18.8;
static const float COUNTS_PER_REV = ENCODER_CPR * GEAR_RATIO / 3.9;

#define SPEED_SAMPLE_MS  20
const float MAX_RPM = 540.0;

// ---------------- PID gain sets ----------------
struct PIDGains { float Kp, Ki, Kd; };
PIDGains pidHigh = {2.0, 30.0, 0.06};
PIDGains pidMid  = {2.25, 12.0, 0.03};
PIDGains pidLow  = {2.5, 9.0, 0.005};

// ---------------- State ----------------
volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastControlTime = 0;

float integral = 0.0;
float lastError = 0.0;
float targetRPM = 0.0;

// =====================================================
// WASD control
// =====================================================
enum ControlCmd {
  CMD_NONE,
  CMD_FORWARD,
  CMD_BACKWARD,
  CMD_LEFT,
  CMD_RIGHT
};

volatile ControlCmd currentCmd = CMD_NONE;

const unsigned long SERVO_UPDATE_MS = 20; // 50Hz
const int SERVO_STEP = 3.0;
unsigned long lastServoUpdate = 0;
const float RPM_STEP = 0.2;
const float MAX_TARGET_RPM = 500.0;

// =====================================================
// Web page (WASD)
// =====================================================
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ESP32 WASD Control</title>
<style>
body {
  background:#111;
  color:#fff;
  text-align:center;
  font-family:Arial;
  padding-top:40px;
}
.key {
  display:inline-block;
  width:60px;
  height:60px;
  line-height:60px;
  margin:5px;
  border:2px solid #0af;
  border-radius:8px;
}
</style>
</head>
<body>

<h2>ESP32 WASD Remote</h2>

<div>
  <div class="key">W</div><br>
  <div class="key">A</div>
  <div class="key">S</div>
  <div class="key">D</div>
</div>

<script>
let pressed = {};

document.addEventListener("keydown", e => {
  if (pressed[e.key]) return;
  pressed[e.key] = true;
  fetch(`/cmd?key=${e.key}`);
});

document.addEventListener("keyup", e => {
  pressed[e.key] = false;
  fetch(`/cmd?key=none`);
});
</script>

</body>
</html>
)rawliteral";

// =====================================================
// Encoder ISR
// =====================================================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B_PIN)) encoderCount++;
  else encoderCount--;
}

// =====================================================
// HTTP handlers
// =====================================================
void handleRoot() {
  server.send(200, "text/html", PAGE);
}

void handleCmd() {
  if (!server.hasArg("key")) {
    server.send(400, "text/plain", "Missing key");
    return;
  }

  String k = server.arg("key");

  if (k == "w") currentCmd = CMD_FORWARD;
  else if (k == "s") currentCmd = CMD_BACKWARD;
  else if (k == "a") currentCmd = CMD_LEFT;
  else if (k == "d") currentCmd = CMD_RIGHT;
  else currentCmd = CMD_NONE;

  server.send(200, "text/plain", "OK");
}

// =====================================================
// Setup
// =====================================================
void setup() {
  Serial.begin(115200);

  // Servo
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(servoAngle);

  // Motor PWM
  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  stopMotor();

  // Encoder
  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet, dns1, dns2);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(300);

  // Web server
  server.on("/", handleRoot);
  server.on("/cmd", handleCmd);
  server.begin();

  lastControlTime = millis();
}

// =====================================================
// Main loop
// =====================================================
void loop() {
  server.handleClient();
  handleWASD();

  unsigned long now = millis();
  if (now - lastControlTime >= SPEED_SAMPLE_MS) {
    float currentRPM = computeRPM(now - lastControlTime);
    applySpeedControl(targetRPM, currentRPM, (now - lastControlTime) / 1000.0);

    Serial.print("target_rpm:"); 
    Serial.print(targetRPM); 
    Serial.print(",current_rpm:"); 
    Serial.print(currentRPM); 
    Serial.print(",ymin:-550,ymax:550"); 
    Serial.println();

    lastControlTime = now;
  }
}

// =====================================================
// WASD logic
// =====================================================
void handleWASD() {
  // ---- Motor speed ----
  if (currentCmd == CMD_FORWARD) {
    targetRPM = constrain(targetRPM + RPM_STEP, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  } 
  else if (currentCmd == CMD_BACKWARD) {
    targetRPM = constrain(targetRPM - RPM_STEP, -MAX_TARGET_RPM, MAX_TARGET_RPM);
  }

  // ---- Servo (fixed rate update) ----
  unsigned long now = millis();
  if (now - lastServoUpdate >= SERVO_UPDATE_MS) {
    if (currentCmd == CMD_LEFT) {
      servoAngle = constrain(servoAngle - SERVO_STEP, 0, 180);
      servo.write(servoAngle);
    } 
    else if (currentCmd == CMD_RIGHT) {
      servoAngle = constrain(servoAngle + SERVO_STEP, 0, 180);
      servo.write(servoAngle);
    }
    lastServoUpdate = now;
  }
}

// =====================================================
// Speed computation + control
// =====================================================
float computeRPM(unsigned long dtMs) {
  long delta = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;
  return (delta / COUNTS_PER_REV) / (dtMs / 1000.0) * 60.0;
}

// =====================================================
// PID gain selection
// =====================================================
PIDGains selectPID(float target) {
  float absRPM = fabs(target);
  if (absRPM > 200.0) return pidHigh;
  if (absRPM > 100.0) return pidMid;
  return pidLow;
}

// =====================================================
// Speed control
// =====================================================
void applySpeedControl(float target, float current, float dt) {
  if (fabs(target) < 50.0) {
    handleVeryLowSpeed(target, current);
    return;
  }

  PIDGains g = selectPID(target);
  float error = target - current;

  integral += error * dt;
  integral = constrain(integral, -MAX_RPM, MAX_RPM);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float output = g.Kp * error + g.Ki * integral + g.Kd * derivative;
  output = constrain(output, -MAX_RPM, MAX_RPM);
  setMotorRPMOutput(output);
}

// =====================================================
// Low-speed special handling
// =====================================================
void handleVeryLowSpeed(float target, float current) {
  const int BASE_PWM_POS = 40;
  const int BASE_PWM_NEG = 45;
  const int PWM_STEP = 10;
  const float DEAD_RPM = 5.0;

  if (fabs(target) < 5.0) {
    stopMotor();
    integral = 0;
    lastError = 0;
    return;
  }

  bool forward = target > 0;
  int base = forward ? BASE_PWM_POS : BASE_PWM_NEG;

  if (current < target - DEAD_RPM) {
    setMotorPWMDirect(forward ? base + PWM_STEP : -(base + PWM_STEP));
  } 
  else if (current > target + DEAD_RPM) {
    setMotorPWMDirect(forward ? base - PWM_STEP : -(base - PWM_STEP));
  } 
  else {
    setMotorPWMDirect(forward ? base : -base);
  }
}

// =====================================================
// Motor helpers
// =====================================================
void setMotorRPMOutput(float rpmCmd) {
  int pwm = fabs(rpmCmd) / MAX_RPM * PWM_MAX;
  pwm = constrain(pwm, 0, PWM_MAX);
  setMotorPWMDirect(rpmCmd >= 0 ? pwm : -pwm);
}

void setMotorPWMDirect(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm >= 0) {
    ledcWrite(RPWM_PIN, pwm);
    ledcWrite(LPWM_PIN, 0);
  } else {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, -pwm);
  }
}

void stopMotor() {
  ledcWrite(RPWM_PIN, 0);
  ledcWrite(LPWM_PIN, 0);
}
