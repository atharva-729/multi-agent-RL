#include <Arduino.h> // Standard Arduino library
#define POSITION_TOLERANCE_MM 10.0
#define MIN_MOVABLE_RPM 37.5

// --- Bluetooth Module Pins (HC-05 using Hardware Serial1 on Mega) ---
// On Arduino Mega:
// RX1 (Pin 19) will receive data from HC-05 TX
// TX1 (Pin 18) will send data to HC-05 RX (via voltage divider)
// Ensure your HC-05's baud rate is set to 9600 (default) or match it here.

// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 9;  // PWM pin for Left Motor speed (Connect to L298N ENB)
const int IN1L = 8;  // Left Motor Direction Pin 1 (Connect to L298N IN3)
const int IN2L = 7;  // Left Motor Direction Pin 2 (Connect to L298N IN4)

// Right Motor pins
const int ENAR = 10; // PWM pin for Right Motor speed (Connect to L298N ENA)
const int IN3R = 5;  // Right Motor Direction Pin 1 (Connect to L298N IN1)
const int IN4R = 4;  // Right Motor Direction Pin 2 (Connect to L298N IN2)

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16; // Pulses Per Revolution of your encoder
const unsigned long DEBOUNCE_DELAY_MS = 20; // Debounce for encoder interrupts
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// --- Timing for RPM calculation and PID loop ---
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 200ms

// --- PID Control Variables for LEFT Motor ---
float Kp_L = 0.6;  // Proportional gain for Left Motor (adjust as needed)
float Ki_L = 0.0;  // Integral gain for Left Motor (start with 0.0, tune later)
float Kd_L = 0.2;  // Derivative gain for Left Motor (start with 0.0, tune later)

float targetRPM_Left = 75.0; // Desired target RPM for Left Motor
float currentRPM_Left = 0.0;
float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;
float outputPWM_Left = 0.0; // This variable will be cumulatively updated

// --- PID Control Variables for RIGHT Motor ---
float Kp_R = 0.55;  // Proportional gain for Right Motor
float Ki_R = 0.0;  // Integral gain for Right Motor
float Kd_R = 0.3;  // Derivative gain for Right Motor

float targetRPM_Right = 75.0; // Desired target RPM for Right Motor
float currentRPM_Right = 0.0;
float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;
float outputPWM_Right = 0.0; // This variable will be cumulatively updated

// --- Position Control Parameters ---
const float WHEEL_CIRCUM_MM = 219.9; // 70 mm dia
float currentPos_mm = 0.0;
float targetPos_mm = 600.0; // <-- You can change this later via Bluetooth if desired
float posError = 0.0;
float prevPosError = 0.0;
float Kp_pos = 0.17;  // You will tune this
float Kd_pos = 0.0;  // Optional
float Ki_pos = 0.0;  // really not needed

float prevTargetRPM_Left = 0.0;
float prevTargetRPM_Right = 0.0;

// --- Motor Characterization (Regression Lines) ---
const float REGRESSION_SLOPE_L = 0.4523;
const float REGRESSION_INTERCEPT_L = -3.0138;

const float REGRESSION_SLOPE_R = 0.3636;
const float REGRESSION_INTERCEPT_R = 12.2727;

// --- PWM Output Limits ---
const int MIN_PWM_OVERALL = 0; // Overall minimum PWM (0 for off)
const int MAX_PWM_OVERALL = 255; // Overall maximum PWM

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
const int MIN_EFFECTIVE_PWM_L = 85; // For Left Motor (adjust if needed for 9V)
const int MIN_EFFECTIVE_PWM_R = 85; // For Right Motor (adjust if needed for 9V)


void setup() {
  // Initialize USB Serial for debugging (to your laptop's Arduino IDE Serial Monitor)
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open (useful for some boards)

  // Initialize Hardware Serial1 for HC-05 Bluetooth communication
  Serial1.begin(9600); // HC-05 default baud rate is 9600
  Serial1.println("Arduino: HC-05 Ready for Data Streaming!");

  // Motor Driver Pin Setup for LEFT Motor
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Motor Driver Pin Setup for RIGHT Motor
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder Pin Setup for BOTH Encoders
  // Digital Pin 3 (Interrupt 1) and Digital Pin 2 (Interrupt 0) are external interrupts on Arduino Mega
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP); // Use INPUT_PULLUP if encoder outputs open-collector/open-drain
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE); // Trigger on any state change
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize both motors to stop
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
  delay(15000); // Short delay to allow everything to initialize
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // --- Step 1: Read Encoder Ticks Safely ---
    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    long currentRightTicks_copy = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    // --- Step 2: Update Position (average of both wheels) ---
    float deltaRevLeft = (float)currentLeftTicks_copy / (float)PPR;
    float deltaRevRight = (float)currentRightTicks_copy / (float)PPR;

    float deltaDistLeft = deltaRevLeft * WHEEL_CIRCUM_MM;
    float deltaDistRight = deltaRevRight * WHEEL_CIRCUM_MM;

    float deltaDistance = (deltaDistLeft + deltaDistRight) / 2.0;
    currentPos_mm += deltaDistance;

    // --- Step 3: Check if Target Reached ---
    if (currentPos_mm >= targetPos_mm) {
      targetRPM_Left = 0;
      targetRPM_Right = 0;

      outputPWM_Left = 0;
      outputPWM_Right = 0;

      analogWrite(ENAL, 0);
      analogWrite(ENAR, 0);
      digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW);
      digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW);

      Serial1.print(currentTime);
      Serial1.print(",");
      Serial1.print(currentRPM_Left);
      Serial1.print(",");
      Serial1.print(0);
      Serial1.print(",");
      Serial1.print(currentRPM_Right);
      Serial1.print(",");
      Serial1.println(0);
      return;
    }

    // --- Step 4: Outer Loop PID (Position to Velocity) ---
    float posError = targetPos_mm - currentPos_mm;
    float dError_pos = (posError - prevPosError) / (deltaTime_ms / 1000.0);
    prevPosError = posError;

    float targetVelRPM = Kp_pos * posError + Kd_pos * dError_pos;

    // --- Clamp velocity range ---
    if (targetVelRPM < 0) targetVelRPM = 0;
    if (targetVelRPM > 150) targetVelRPM = 150;

    // --- Quantize ---
    targetVelRPM = round(targetVelRPM / 18.75) * 18.75;

    // --- Prevent slow crawling ---
    if (targetVelRPM > 0 && targetVelRPM < MIN_MOVABLE_RPM) {
      targetVelRPM = MIN_MOVABLE_RPM;
    }

    // --- Final stop condition: within position tolerance ---
    if (abs(posError) < POSITION_TOLERANCE_MM) {
      targetVelRPM = 0; // Close enough, stop commanding motion
    }

    targetRPM_Left = targetVelRPM;
    targetRPM_Right = targetVelRPM;


    // --- Step 5: Velocity PID (Left) ---
    currentRPM_Left = deltaRevLeft / (deltaTime_ms / 60000.0);
    float error_L = targetRPM_Left - currentRPM_Left;
    float basePWM_L = (targetRPM_Left - REGRESSION_INTERCEPT_L) / REGRESSION_SLOPE_L;
    if (targetRPM_Left != prevTargetRPM_Left && targetRPM_Left > 0) {
      outputPWM_Left = basePWM_L;
      prevTargetRPM_Left = targetRPM_Left;
    }

    float P_L = Kp_L * error_L;
    if (Ki_L > 0) integral_Left += error_L * (deltaTime_ms / 1000.0);
    float I_L = Ki_L * integral_Left;
    float D_L = Kd_L * (error_L - prevError_Left) / (deltaTime_ms / 1000.0);
    prevError_Left = error_L;
    outputPWM_Left += P_L + I_L + D_L;
    if (targetRPM_Left == 0) outputPWM_Left = 0;
    else if (outputPWM_Left < MIN_EFFECTIVE_PWM_L) outputPWM_Left = MIN_EFFECTIVE_PWM_L;
    if (outputPWM_Left > 255) outputPWM_Left = 255;
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left);

    // --- Step 6: Velocity PID (Right) ---
    currentRPM_Right = deltaRevRight / (deltaTime_ms / 60000.0);
    float error_R = targetRPM_Right - currentRPM_Right;
    float basePWM_R = (targetRPM_Right - REGRESSION_INTERCEPT_R) / REGRESSION_SLOPE_R;
    if (targetRPM_Right != prevTargetRPM_Right && targetRPM_Right > 0) {
      outputPWM_Right = basePWM_R;
      prevTargetRPM_Right = targetRPM_Right;
    }

    float P_R = Kp_R * error_R;
    if (Ki_R > 0) integral_Right += error_R * (deltaTime_ms / 1000.0);
    float I_R = Ki_R * integral_Right;
    float D_R = Kd_R * (error_R - prevError_Right) / (deltaTime_ms / 1000.0);
    prevError_Right = error_R;
    outputPWM_Right += P_R + I_R + D_R;
    if (targetRPM_Right == 0) outputPWM_Right = 0;
    else if (outputPWM_Right < MIN_EFFECTIVE_PWM_R) outputPWM_Right = MIN_EFFECTIVE_PWM_R;
    if (outputPWM_Right > 255) outputPWM_Right = 255;
    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right);

    // --- Data Output ---
    Serial1.print("Left RPM: ");
    Serial1.print(currentRPM_Left);
    Serial1.print(" | Left PWM: ");
    Serial1.print((int)outputPWM_Left);
    Serial1.print(" | Right RPM: ");
    Serial1.print(currentRPM_Right);
    Serial1.print(" | Right PWM: ");
    Serial1.print((int)outputPWM_Right);
    Serial1.print(" | Position: ");
    Serial1.print(currentPos_mm);
    Serial1.print(" mm | Position Error: ");
    Serial1.println(posError);
    
  }
}

// --- Encoder Interrupt Service Routine (ISR) for LEFT Encoder ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++;
    lastPulseTimeLeft = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for RIGHT Encoder ---
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}
