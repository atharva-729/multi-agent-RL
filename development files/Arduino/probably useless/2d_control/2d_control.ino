#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define POSITION_TOLERANCE_MM 10.0
#define ANGLE_TOLERANCE_RAD (2.0 * PI / 180.0)  // 2 degrees in radians
#define MIN_MOVABLE_RPM 37.5

// --- Motor Driver Pins (L298N) ---
const int ENAL = 9;  // PWM pin for Left Motor speed
const int IN1L = 8;  // Left Motor Direction Pin 1
const int IN2L = 7;  // Left Motor Direction Pin 2

const int ENAR = 10; // PWM pin for Right Motor speed
const int IN3R = 5;  // Right Motor Direction Pin 1
const int IN4R = 4;  // Right Motor Direction Pin 2

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;
const int RIGHT_ENCODER_PIN = 2;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

const int PPR = 16; // 8 magnets = 8 pulses per revolution
const unsigned long DEBOUNCE_DELAY_MS = 20;
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// --- Timing ---
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // Faster loop for better control

// --- Robot State ---
enum RobotState {
  IDLE,
  ROTATING,
  MOVING_FORWARD,
  COMPLETED
};

RobotState currentState = IDLE;

// --- Position and Orientation ---
float currentX = 0.0;
float currentY = 0.0;
float currentYaw = 0.0; // Current heading in radians
float yawBias = 0.0;    // Gyroscope bias

float targetX = 0.0;
float targetY = 0.0;
float targetDistance = 0.0;
float targetAngle = 0.0;

// --- Motor Characterization ---
const float WHEEL_CIRCUM_MM = 219.9; // 70mm diameter wheel
const float REGRESSION_SLOPE_L = 0.4523;
const float REGRESSION_INTERCEPT_L = -3.0138;
const float REGRESSION_SLOPE_R = 0.3636;
const float REGRESSION_INTERCEPT_R = 12.2727;

// --- Position Control PID ---
float Kp_pos = 0.17;
float Kd_pos = 0.0;
float Ki_pos = 0.0;

float currentPos_mm = 0.0;
float targetPos_mm = 0.0;
float posError = 0.0;
float prevPosError = 0.0;

// --- Velocity PID for LEFT Motor ---
float Kp_L = 0.6;
float Ki_L = 0.0;
float Kd_L = 0.2;

float targetRPM_Left = 0.0;
float currentRPM_Left = 0.0;
float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float outputPWM_Left = 0.0;
float prevTargetRPM_Left = 0.0;

// --- Velocity PID for RIGHT Motor ---
float Kp_R = 0.55;
float Ki_R = 0.0;
float Kd_R = 0.3;

float targetRPM_Right = 0.0;
float currentRPM_Right = 0.0;
float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float outputPWM_Right = 0.0;
float prevTargetRPM_Right = 0.0;

// --- Rotation Control ---
float kp_rotation = 150.0;
float minPWM_rotation = 155.0;
const float rotation_slope = 0.9368;
const float rotation_intercept = -126.8446;

// --- PWM Limits ---
const int MIN_EFFECTIVE_PWM_L = 85;
const int MIN_EFFECTIVE_PWM_R = 85;

// --- Function Prototypes ---
void calculateYawBias();
void setMotorDirections(int pwmL, int pwmR);
void stopMotors();
void setTarget(float x, float y);
void updatePosition(float deltaTime_ms);
void updateOrientation(float deltaTime_ms);
void controlRotation();
void controlForwardMovement(float deltaTime_ms);
void leftEncoderISR();
void rightEncoderISR();

void setup() {
  // Initialize Bluetooth Serial
  Serial1.begin(9600);
  Serial1.println("Arduino: 2D Robot Control System Ready!");

  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Motor pin setup
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  // Encoder setup
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial1.println("MPU6050 init failed!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Stop motors initially
  stopMotors();
  
  delay(15000); // Wait for MPU6050 to stabilize
  
  calculateYawBias();
  lastLoopTime = millis();
  
  Serial1.println("Setup complete. Send target coordinates as 'X,Y' (e.g., '100,100')");
  setTarget(100.0, 100.0);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    

    // Update robot's orientation from gyroscope
    updateOrientation(deltaTime_ms);

    // State machine for 2D navigation
    switch (currentState) {
      case IDLE:
        stopMotors();
        break;

      case ROTATING:
        controlRotation();
        break;

      case MOVING_FORWARD:
        updatePosition(deltaTime_ms);
        controlForwardMovement(deltaTime_ms);
        break;

      case COMPLETED:
        stopMotors();
        Serial1.print("Target reached! Current position: (");
        Serial1.print(currentX);
        Serial1.print(", ");
        Serial1.print(currentY);
        Serial1.println(")");
        currentState = IDLE;
        break;
    }

    // Output telemetry
    Serial1.print("State:");
    Serial1.print(currentState);
    Serial1.print(",X:");
    Serial1.print(currentX);
    Serial1.print(",Y:");
    Serial1.print(currentY);
    Serial1.print(",Yaw:");
    Serial1.print(currentYaw * 180.0 / PI);
    Serial1.print(",L_RPM:");
    Serial1.print(currentRPM_Left);
    Serial1.print(",R_RPM:");
    Serial1.println(currentRPM_Right);
  }
}

void setTarget(float x, float y) {
  if (currentState != IDLE) {
    Serial1.println("Robot busy. Wait for completion.");
    return;
  }

  targetX = x;
  targetY = y;
  
  // Calculate polar coordinates
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;
  
  targetDistance = sqrt(deltaX * deltaX + deltaY * deltaY);
  targetAngle = atan2(deltaY, deltaX);
  
  // Normalize angle difference
  float angleDiff = targetAngle - currentYaw;
  while (angleDiff > PI) angleDiff -= 2 * PI;
  while (angleDiff < -PI) angleDiff += 2 * PI;
  
  Serial1.print("New target: (");
  Serial1.print(x);
  Serial1.print(", ");
  Serial1.print(y);
  Serial1.print(") Distance: ");
  Serial1.print(targetDistance);
  Serial1.print("mm, Angle: ");
  Serial1.print(targetAngle * 180.0 / PI);
  Serial1.println(" degrees");
  
  if (abs(angleDiff) > ANGLE_TOLERANCE_RAD) {
    Serial1.println("Starting rotation...");
    currentState = ROTATING;
  } else if (targetDistance > POSITION_TOLERANCE_MM) {
    Serial1.println("Starting forward movement...");
    currentPos_mm = 0.0; // Reset position counter
    targetPos_mm = targetDistance;
    currentState = MOVING_FORWARD;
  } else {
    Serial1.println("Already at target!");
    currentState = COMPLETED;
  }
}

void updateOrientation(float deltaTime_ms) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float correctedGz = g.gyro.z - yawBias;
  currentYaw += correctedGz * (deltaTime_ms / 1000.0);
  
  // Keep yaw in [-PI, PI] range
  while (currentYaw > PI) currentYaw -= 2 * PI;
  while (currentYaw < -PI) currentYaw += 2 * PI;
}

void updatePosition(float deltaTime_ms) {
  // Read encoder ticks safely
  noInterrupts();
  long currentLeftTicks_copy = leftEncoderTicks;
  long currentRightTicks_copy = rightEncoderTicks;
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  interrupts();

  // Calculate distance traveled
  float deltaRevLeft = (float)currentLeftTicks_copy / (float)PPR;
  float deltaRevRight = (float)currentRightTicks_copy / (float)PPR;

  float deltaDistLeft = deltaRevLeft * WHEEL_CIRCUM_MM;
  float deltaDistRight = deltaRevRight * WHEEL_CIRCUM_MM;

  float deltaDistance = (deltaDistLeft + deltaDistRight) / 2.0;
  currentPos_mm += deltaDistance;

  // Update global position
  currentX += deltaDistance * cos(currentYaw);
  currentY += deltaDistance * sin(currentYaw);

  // Calculate current RPM
  currentRPM_Left = deltaRevLeft / (deltaTime_ms / 60000.0);
  currentRPM_Right = deltaRevRight / (deltaTime_ms / 60000.0);
}

void controlRotation() {
  float error = targetAngle - currentYaw;
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;

  if (abs(error) < ANGLE_TOLERANCE_RAD) {
    stopMotors();
    Serial1.println("Rotation complete. Starting forward movement...");
    currentPos_mm = 0.0;
    targetPos_mm = targetDistance;
    currentState = MOVING_FORWARD;
    return;
  }

  // Convert angular error to desired yaw rate
  float desiredGz = kp_rotation * error;
  desiredGz = constrain(desiredGz, -150, 150);

  // Compute PWM from linear regression
  float pwm = (desiredGz - rotation_intercept) / rotation_slope;

  // Apply minimum PWM constraint
  if (abs(pwm) < minPWM_rotation) {
    pwm = (pwm > 0) ? minPWM_rotation : -minPWM_rotation;
  }

  pwm = constrain(pwm, -255, 255);
  setMotorDirections(-pwm, pwm); // Rotate in place
}

void controlForwardMovement(float deltaTime_ms) {
  // Check if target position reached
  if (currentPos_mm >= targetPos_mm) {
    stopMotors();
    Serial1.println("Forward movement complete!");
    currentState = COMPLETED;
    return;
  }

  // Position PID to velocity
  float posError = targetPos_mm - currentPos_mm;
  float dError_pos = (posError - prevPosError) / (deltaTime_ms / 1000.0);
  prevPosError = posError;

  float targetVelRPM = Kp_pos * posError + Kd_pos * dError_pos;

  // Clamp velocity
  if (targetVelRPM < 0) targetVelRPM = 0;
  if (targetVelRPM > 150) targetVelRPM = 150;

  // Quantize velocity
  targetVelRPM = round(targetVelRPM / 18.75) * 18.75;

  // Prevent slow crawling
  if (targetVelRPM > 0 && targetVelRPM < MIN_MOVABLE_RPM) {
    targetVelRPM = MIN_MOVABLE_RPM;
  }

  // Final stop condition
  if (abs(posError) < POSITION_TOLERANCE_MM) {
    targetVelRPM = 0;
  }

  targetRPM_Left = targetVelRPM;
  targetRPM_Right = targetVelRPM;

  // Left motor velocity PID
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

  // Right motor velocity PID
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

  // Set motor directions for forward movement
  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
  digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
  analogWrite(ENAL, (int)outputPWM_Left);
  analogWrite(ENAR, (int)outputPWM_Right);
}

void setMotorDirections(int pwmL, int pwmR) {
  if (pwmL >= 0) {
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
  } else {
    digitalWrite(IN1L, LOW); digitalWrite(IN2L, HIGH);
    pwmL = -pwmL;
  }

  if (pwmR >= 0) {
    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
  } else {
    digitalWrite(IN3R, LOW); digitalWrite(IN4R, HIGH);
    pwmR = -pwmR;
  }

  analogWrite(ENAL, constrain(pwmL, 0, 255));
  analogWrite(ENAR, constrain(pwmR, 0, 255));
}

void stopMotors() {
  analogWrite(ENAL, 0);
  analogWrite(ENAR, 0);
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW);
  
  // Reset PID variables
  integral_Left = 0;
  integral_Right = 0;
  outputPWM_Left = 0;
  outputPWM_Right = 0;
}

void calculateYawBias() {
  Serial1.println("Calculating gyroscope bias...");
  float sum = 0.0;
  sensors_event_t a, g, temp;
  
  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  
  yawBias = sum / 200.0;
  Serial1.print("Yaw bias calculated: ");
  Serial1.println(yawBias, 6);
}

void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++;
    lastPulseTimeLeft = currentTime;
  }
}

void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}