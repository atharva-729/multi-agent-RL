#include <Arduino.h> // Standard Arduino library
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define POSITION_TOLERANCE_MM 15.0
#define MIN_MOVABLE_RPM 37.5

Adafruit_MPU6050 mpu;

enum RobotState {
  IDLE,
  ROTATING,
  MOVING_FORWARD,
  COMPLETED
};

RobotState currentState = IDLE;

// L298N Motor Driver Pins
const int ENAL = 9;
const int IN1L = 8;
const int IN2L = 7;

const int ENAR = 10;
const int IN3R = 5;
const int IN4R = 4;

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

float yawAngle = 0.0;
float yawBias = 0.0;
unsigned long lastTime = 0;
float lastYawError = 0.0;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16; // Pulses Per Revolution of your encoder
const unsigned long DEBOUNCE_DELAY_MS = 20; // Debounce for encoder interrupts
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 200ms

long currentLeftTicks_copy = 0;
long currentRightTicks_copy = 0;
float deltaRevLeft = 0.0;
float deltaRevRight = 0.0;
float deltaDistLeft = 0.0;
float deltaDistRight = 0.0;
float deltaDistance = 0.0;

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
float outputPWM_Right = 0.0; 

// --- Position Control Parameters ---
const float WHEEL_CIRCUM_MM = 219.9; // 70 mm dia
float currentPos_mm = 0.0;
float targetPos_mm = 0.0; // <-- You can change this later via Bluetooth if desired
float posError = 0.0;
float prevPosError = 0.0;
float Kp_pos = 0.17;  // You will tune this
float Kd_pos = 0.0;  // Optional
float Ki_pos = 0.0;  // really not needed

float targetAngle = 0.0; //45.0 * (PI / 180.0);  // rotate 90 degrees
float kpyaw = 100;        // proportional gain
float kdyaw = 25;
float minPWM = 150.0;  // minimum PWM to overcome static friction

float prevTargetRPM_Left = 0.0;
float prevTargetRPM_Right = 0.0;

// Linear regression model: Gz = slope * PWM + intercept
const float slope = 0.9368;
const float intercept = -126.8446;

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

// --- Position and Orientation ---
float currentX = 0.0;
float currentY = 0.0;
float currentYaw = 0.0; // Current heading in radians

float targetX = 0.0;
float targetY = 0.0;
float targetDistance = 0.0;
// float targetAngle = 0.0;

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
  delay(10);;

  analogWrite(ENAL, constrain(pwmL, 0, 255));
  analogWrite(ENAR, constrain(pwmR, 0, 255));
}

void stopMotors() {
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
}

void calculateYawBias() {
  float sum = 0.0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  yawBias = sum / 200.0;
  Serial1.print("Yaw bias: ");
  Serial1.println(yawBias, 6);
}




///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////


void setup() {
  Wire.begin();
  Serial1.begin(9600);
  Serial1.println("Arduino: HC-05 Ready for Data Streaming!");

  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  if (!mpu.begin()) {
    Serial1.println("MPU6050 init failed!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP); // Use INPUT_PULLUP if encoder outputs open-collector/open-drain
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE); // Trigger on any state change
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize both motors to stop
  stopMotors();
  
  delay(15000);

  calculateYawBias();
  lastTime = millis();
  Serial1.println("Setup done. Starting rotation.");
}


///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
float x_coord = 0.0;
float y_coord = 0.0;


void loop() {

  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    input.trim();
    
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      x_coord = input.substring(0, commaIndex).toFloat();
      y_coord = input.substring(commaIndex + 1).toFloat();
      currentState = IDLE; // remove this after demo
  }
  delay(100);
}
  if (!(x_coord == 0.0 && y_coord == 0.0)) {
    // State machine for 2D navigation
    switch (currentState) {
      case IDLE:
        stopMotors();
        getPolar(x_coord, y_coord);
        break;

      case ROTATING:
        controlRotation();
        leftEncoderTicks = 0;
        rightEncoderTicks = 0;
        break;

      case MOVING_FORWARD:
        controlForward();
        break;

      case COMPLETED:
        stopMotors();
        Serial1.print("Target reached!"); // Current position: (");
        // Serial1.print(currentX);
        // Serial1.print(", ");
        // Serial1.print(currentY);
        // Serial1.println(")");
        break;
    }
  }

}

void getPolar(float x, float y) {
  targetX = x;
  targetY = y;

    // Calculate polar coordinates
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;
  
  targetPos_mm = sqrt(deltaX * deltaX + deltaY * deltaY);
  targetAngle = atan2(deltaY, deltaX);
  
  // // Normalize angle difference
  // float angleDiff = targetAngle - currentYaw;
  // while (angleDiff > PI) angleDiff -= 2 * PI;
  // while (angleDiff < -PI) angleDiff += 2 * PI;
  
  Serial1.print("New target: (");
  Serial1.print(x);
  Serial1.print(", ");
  Serial1.print(y);
  Serial1.print(") Distance: ");
  Serial1.print(targetPos_mm);
  Serial1.print("mm, Angle: ");
  Serial1.print(targetAngle * 180.0 / PI);
  Serial1.println(" degrees");
  currentState = ROTATING;
  delay(50);
  return;
}


void controlRotation() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float correctedGz = g.gyro.z - yawBias;
  yawAngle += correctedGz * dt;

  float error = targetAngle - yawAngle;
  while (error > PI) error -= 2 * PI;
  while (error < -PI) error += 2 * PI;

  if (abs(error) < 3.0 * PI / 180.0) {  // ~2 degree tolerance
    stopMotors();
    Serial1.println("Rotation complete. Starting forward movement.");
    Serial1.print("Yaw (deg): ");
    Serial1.print(yawAngle * 180.0 / PI);
    Serial1.print(" | Error: ");
    Serial1.println(error * 180.0 / PI);
    currentState = MOVING_FORWARD;
    delay(1000);
    return;
  }

  //   if (abs(yawAngle) > abs(targetAngle)) {
  //   stopMotors();
  //   Serial1.println("Did not stop at target. Overshot. Had to stop.");
  //   Serial1.print("Yaw (deg): ");
  //   Serial1.print(yawAngle * 180.0 / PI);
  //   Serial1.print(" | Error: ");
  //   Serial1.println(error * 180.0 / PI);
  //   currentState = MOVING_FORWARD;
  //   delay(1000);
  //   return;
  // }

  // Convert current angular error to deg/s desired yaw rate
  float kdTerm = kdyaw * (error - lastYawError) / dt;
  lastYawError = error;

  float desiredGz = kpyaw * error + kdTerm;  // deg/s because correctedGz is in rad/s

  // Cap desiredGz to a reasonable max (you can tune this)
  desiredGz = constrain(desiredGz, -150, 150);

  // Compute base PWM from linear regression
  float pwm = (abs(desiredGz) - intercept) / slope;

  if (desiredGz < 0) pwm = -pwm;

  // Apply minimum PWM constraint
  if (abs(pwm) < minPWM) {
    pwm = (pwm > 0) ? minPWM : -minPWM;
  }

  pwm = constrain(pwm, -255, 255);
  delay(50);
  setMotorDirections(-pwm, pwm);  // rotate in place
  delay(50);

  // Debug output
  Serial1.print("Yaw (deg): ");
  Serial1.print(yawAngle * 180.0 / PI);
  Serial1.print(" | Error: ");
  Serial1.print(error * 180.0 / PI);
  Serial1.print(" | Gz corr: ");
  Serial1.print(correctedGz * 180.0 / PI);
  Serial1.print(" | PWM: ");
  Serial1.println(pwm);

  delay(20);
}

void controlForward() {
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
    deltaRevLeft = (float)currentLeftTicks_copy / (float)PPR;
    deltaRevRight = (float)currentRightTicks_copy / (float)PPR;

    deltaDistLeft = deltaRevLeft * WHEEL_CIRCUM_MM;
    deltaDistRight = deltaRevRight * WHEEL_CIRCUM_MM;

    deltaDistance = (deltaDistLeft + deltaDistRight) / 2.0;
    // Serial1.print("currentPos_mm: ");
    // Serial1.print(currentPos_mm);
    // Serial1.print(" | deltaDistance: ");
    // Serial1.println(deltaDistance);
    currentPos_mm += deltaDistance;

    // --- Step 3: Check if Target Reached ---
    if (abs(currentPos_mm - targetPos_mm) <= POSITION_TOLERANCE_MM) {   //abs
      targetRPM_Left = 0;
      targetRPM_Right = 0;

      outputPWM_Left = 0;
      outputPWM_Right = 0;

      stopMotors();

      // Serial1.print(currentTime);
      // Serial1.print(",");
      Serial1.print(currentRPM_Left);
      Serial1.print(",");
      Serial1.print(0);
      Serial1.print(",");
      Serial1.print(currentRPM_Right);
      Serial1.print(",");
      Serial1.println(0);
      // targetAngle = 0.0;
      // controlRotation();
      currentState = COMPLETED;
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
    Serial1.print("Time: ");
    Serial1.print(currentTime);
    Serial1.print(" ms | Left RPM: ");
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
