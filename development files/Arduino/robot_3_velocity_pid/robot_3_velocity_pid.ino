#include <Arduino.h>

// --- Motor Driver Pins ---
const int ENAL = 10;
const int IN1L = 5;
const int IN2L = 4;

const int ENAR = 9;
const int IN3R = 8;
const int IN4R = 7;

// --- Encoder Pins (Analog Read) ---
const int LEFT_ANALOG_PIN = A0;
const int RIGHT_ANALOG_PIN = A1;
const int THRESHOLD = 500;
const int PPR = 8;

// --- Encoder state ---
int prevLeft = 0, prevRight = 0;
long pulsesLeft = 0, pulsesRight = 0;
float currentRPM_Left = 0.0, currentRPM_Right = 0.0;

// --- PID Variables ---
float targetRPM_Left = 93.75, targetRPM_Right = 112.5;

// --- PID tuning ---
float Kp_L = 0.7, Kd_L = 0.25;
float Kp_R = 1.0, Kd_R = 0.25;

float error_Left = 0.0, prevError_Left = 0.0, derivative_Left = 0.0;
float error_Right = 0.0, prevError_Right = 0.0, derivative_Right = 0.0;

float outputPWM_Left = 0.0, outputPWM_Right = 0.0;

// --- Regression Feedforward ---
const float REGRESSION_SLOPE_L = 0.7528;
const float REGRESSION_INTERCEPT_L = 10.2685;

const float REGRESSION_SLOPE_R = 0.7180;
const float REGRESSION_INTERCEPT_R = 11.3389;

// --- Limits ---
const int MAX_PWM = 255;
const int MIN_EFFECTIVE_PWM = 50;
const int MIN_PWM = 0;

// --- Timers ---
unsigned long lastPIDTime = 0;
unsigned long lastRPMTime = 0;
const unsigned long PID_INTERVAL = 400; // ms

void setup() {
  Serial1.begin(9600);
  Serial1.println("Edge-based Analog Encoder PID System Ready");

  // Motor setup
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  pinMode(LEFT_ANALOG_PIN, INPUT);
  pinMode(RIGHT_ANALOG_PIN, INPUT);

  delay(15000);  // Optional delay before motion
}

void loop() {
  unsigned long now = millis();

  // --- Rising edge detection ---
  int leftVal = analogRead(LEFT_ANALOG_PIN);
  int rightVal = analogRead(RIGHT_ANALOG_PIN);

  if (leftVal > THRESHOLD && prevLeft <= THRESHOLD) pulsesLeft++;
  if (rightVal > THRESHOLD && prevRight <= THRESHOLD) pulsesRight++;

  prevLeft = leftVal;
  prevRight = rightVal;

  // --- RPM calculation every 1000 ms ---
  if (now - lastRPMTime >= PID_INTERVAL) {
    currentRPM_Left = (float)pulsesLeft / PPR / (PID_INTERVAL / 60000.0);
    currentRPM_Right = (float)pulsesRight / PPR / (PID_INTERVAL / 60000.0);

    // Serial1.print("RPM_Left: ");
    Serial1.print(now);
    Serial1.print(",");
    Serial1.print(currentRPM_Left);
    Serial1.print(",");
    Serial1.print((int)outputPWM_Left);

    Serial1.print(",");
    Serial1.print(currentRPM_Right);
    Serial1.print(",");
    Serial1.println((int)outputPWM_Right);

    pulsesLeft = 0;
    pulsesRight = 0;
    lastRPMTime = now;
  }

  // --- PID control every 200 ms ---
  if (now - lastPIDTime >= PID_INTERVAL) {
    float deltaTime = (now - lastPIDTime) / 1000.0;
    lastPIDTime = now;

    // --- Left Motor PID ---
    error_Left = targetRPM_Left - currentRPM_Left;
    derivative_Left = (error_Left - prevError_Left) / deltaTime;
    prevError_Left = error_Left;

    float basePWM_L = (targetRPM_Left - REGRESSION_INTERCEPT_L) / REGRESSION_SLOPE_L;
    if (outputPWM_Left == 0.0 && targetRPM_Left > 0) {
      outputPWM_Left = basePWM_L;
      Serial1.print("base pwm: ");
      Serial1.println(outputPWM_Left);
    }

    outputPWM_Left += Kp_L * error_Left + Kd_L * derivative_Left;
    outputPWM_Left = constrain(outputPWM_Left, MIN_PWM, MAX_PWM);
    if (targetRPM_Left > 0 && outputPWM_Left < MIN_EFFECTIVE_PWM) outputPWM_Left = MIN_EFFECTIVE_PWM;
    if (targetRPM_Left == 0) outputPWM_Left = 0;

    // --- Right Motor PID ---
    error_Right = targetRPM_Right - currentRPM_Right;
    derivative_Right = (error_Right - prevError_Right) / deltaTime;
    prevError_Right = error_Right;

    float basePWM_R = (targetRPM_Right - REGRESSION_INTERCEPT_R) / REGRESSION_SLOPE_R;
    if (outputPWM_Right == 0.0 && targetRPM_Right > 0) outputPWM_Right = basePWM_R;

    outputPWM_Right += Kp_R * error_Right + Kd_R * derivative_Right;
    outputPWM_Right = constrain(outputPWM_Right, MIN_PWM, MAX_PWM);
    if (targetRPM_Right > 0 && outputPWM_Right < MIN_EFFECTIVE_PWM) outputPWM_Right = MIN_EFFECTIVE_PWM;
    if (targetRPM_Right == 0) outputPWM_Right = 0;

    // --- Apply Motor Outputs (Forward Motion) ---
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left);

    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right);
  }

  // --- Bluetooth STOP command ---
  if (Serial1.available()) {
    String cmd = Serial1.readStringUntil('\n');
    cmd.trim();
    if (cmd == "STOP") {
      targetRPM_Left = 0; targetRPM_Right = 0;
      outputPWM_Left = 0; outputPWM_Right = 0;
      analogWrite(ENAL, 0); analogWrite(ENAR, 0);
      digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW);
      digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW);
      Serial1.println("Motors STOPPED by Bluetooth");
    }
  }
}
