#include <ESP32Servo.h>

// =======================
// Servo
// =======================
static const int SERVO_PIN = 5;
Servo servo;
int servoAngle = 90;

// =======================
// Motor + Encoder
// =======================
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
const float MAX_RPM = 500.0;

// =======================
// PID
// =======================
struct PIDGains { float Kp, Ki, Kd; };
PIDGains pidHigh = {2.0, 30.0, 0.06};
PIDGains pidMid  = {2.25, 12.0, 0.03};
PIDGains pidLow  = {2.5, 9.0, 0.005};

volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastControlTime = 0;

float integral = 0.0;
float lastError = 0.0;
float targetRPM = 0.0;

// =======================
// Encoder ISR
// =======================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B_PIN)) encoderCount++;
  else encoderCount--;
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(115200);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2500);
  servo.write(servoAngle);

  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  stopMotor();

  pinMode(ENC_A_PIN, INPUT);
  pinMode(ENC_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, RISING);

  lastControlTime = millis();
}

// =======================
// Loop
// =======================
void loop() {
  // ---- Serial input ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int sp = line.indexOf(' ');
    if (sp > 0) {
      servoAngle = constrain(line.substring(0, sp).toInt(), 0, 180);
      targetRPM = constrain(line.substring(sp + 1).toFloat(), -MAX_RPM, MAX_RPM);
      servo.write(servoAngle);
    }
  }

  unsigned long now = millis();
  if (now - lastControlTime >= SPEED_SAMPLE_MS) {
    float currentRPM = computeRPM(now - lastControlTime);
    applySpeedControl(targetRPM, currentRPM, (now - lastControlTime) / 1000.0);

    // ---- feedback to Jetson ----
    Serial.println(currentRPM);

    lastControlTime = now;
  }
}

// =======================
// Speed + PID
// =======================
float computeRPM(unsigned long dtMs) {
  long delta = encoderCount - lastEncoderCount;
  lastEncoderCount = encoderCount;
  return (delta / COUNTS_PER_REV) / (dtMs / 1000.0) * 60.0;
}

PIDGains selectPID(float target) {
  float a = fabs(target);
  if (a > 200.0) return pidHigh;
  if (a > 100.0) return pidMid;
  return pidLow;
}

void applySpeedControl(float target, float current, float dt) {
  if (fabs(target) < 5.0) {
    stopMotor();
    integral = lastError = 0;
    return;
  }

  PIDGains g = selectPID(target);
  float error = target - current;
  integral += error * dt;
  integral = constrain(integral, -MAX_RPM, MAX_RPM);

  float derivative = (error - lastError) / dt;
  lastError = error;

  float out = g.Kp * error + g.Ki * integral + g.Kd * derivative;
  out = constrain(out, -MAX_RPM, MAX_RPM);
  setMotorRPMOutput(out);
}

void setMotorRPMOutput(float rpmCmd) {
  int pwm = fabs(rpmCmd) / MAX_RPM * PWM_MAX;
  pwm = constrain(pwm, 0, PWM_MAX);
  setMotorPWMDirect(rpmCmd >= 0 ? pwm : -pwm);
}

void setMotorPWMDirect(int pwm) {
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
