#include <Arduino.h> // Standard Arduino core functions
#include <Wire.h>    // I2C communication
#include <Adafruit_MPU6050.h> // MPU6050 IMU driver
#include <Adafruit_Sensor.h>  // Sensor abstraction layer

// --- Ultrasonic Sensor Pins ---
#define TRIG_LEFT 43
#define ECHO_LEFT 42
#define TRIG_RIGHT 33
#define ECHO_RIGHT 32

// --- Constants for Motion Accuracy and Obstacles ---
#define POSITION_TOLERANCE_MM 20.0           // Tolerance for stopping at a waypoint
#define MIN_MOVABLE_RPM 37.5                 // Below this, robot doesn't move reliably
#define OBSTACLE_THRESHOLD_CM 30            // Distance to consider an obstacle
#define MIN_VALID_DISTANCE 5                // Filter out too-low ultrasonic readings
#define INVALID_READING -1

// --- Sensor Reading Interval Control ---
unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 500; // Time between ultrasonic readings in ms

// --- Obstacle Counters ---
int leftObstacleCounter = 0;   // Count of consecutive obstacles seen on the left
int rightObstacleCounter = 0;  // Same for right

// --- MPU6050 IMU ---
Adafruit_MPU6050 mpu;

// --- Robot Finite State Machine ---
enum RobotState {
  IDLE,              // Doing nothing
  ROTATING,          // Turning in place
  MOVING_FORWARD,    // Moving straight
  COMPLETED          // Finished all waypoints
};

RobotState currentState = IDLE;  // Initial state

// --- Path Waypoints (in mm) ---
float waypoints[][2] = {
  {450.0, 0.0},
  {450.0, 450.0},
  {0.0, 450.0},
  {0.0, 0.0} 
};
const int totalWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);
int currentWaypointIndex = 0;

// --- Motor Driver Pin Definitions (L298N) ---
const int ENAL = 10;
const int IN1L = 5;
const int IN2L = 4;

const int ENAR = 9;
const int IN3R = 8;
const int IN4R = 7;

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;   // Interrupt pin for left encoder
const int RIGHT_ENCODER_PIN = 2;  // Interrupt pin for right encoder

// --- Yaw Control Variables ---
float yawAngle = 0.0;             // Current yaw angle (integrated gyro)
float yawBias = 0.0;              // Offset to correct gyro drift
unsigned long lastTime = 0;
float lastYawError = 0.0;
float yawErrorSum = 0.0;
float lastYaw = 0.0;

// --- Encoder Tick Counters (interrupt updated) ---
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// --- Encoder Specs ---
const int PPR = 16;  // Pulses Per Revolution (adjust if needed)
const unsigned long DEBOUNCE_DELAY_MS = 20; // Encoder debounce

// --- Debounce Timers ---
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// --- Motion Tracking Variables ---
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop update interval

// --- Delta Movement Variables ---
long currentLeftTicks_copy = 0;
long currentRightTicks_copy = 0;
float deltaRevLeft = 0.0;
float deltaRevRight = 0.0;
float deltaDistLeft = 0.0;
float deltaDistRight = 0.0;
float deltaDistance = 0.0;

// --- PID Control Variables for LEFT Motor ---
float Kp_L = 0.9;  // Proportional gain for Left Motor 
float Ki_L = 0.0;  // Integral gain for Left Motor
float Kd_L = 0.15;  // Derivative gain for Left Motor 

float targetRPM_Left = 56.25;        // Setpoint for Left motor RPM
float currentRPM_Left = 0.0;        // Actual measured RPM
float error_Left = 0.0;             
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;
float outputPWM_Left = 0.0;         // Final PWM output after PID

// ======================= PID Control: Right Motor ==========================
float Kp_R = 0.95;  // Proportional gain for Right Motor
float Ki_R = 0.0;  // Integral gain for Right Motor
float Kd_R = 0.0;  // Derivative gain for Right Motor

float targetRPM_Right = 56.25; // Desired target RPM for Right Motor
float currentRPM_Right = 0.0;
float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;
float outputPWM_Right = 0.0; 

// ======================= Position Control ==========================
const float WHEEL_CIRCUM_MM = 219.9; // mm (derived from 70mm diameter wheels)

float currentPos_mm = 0.0;     // Distance traveled so far (in mm)
float targetPos_mm = 0.0;      // Target distance to move (can change via Bluetooth)
float posError = 0.0;
float prevPosError = 0.0;

float Kp_pos = 0.25;
float Kd_pos = 0.05;
float Ki_pos = 0.0;

// ======================= Rotation (Yaw) Control ====================
float targetAngle = 0.0;       // Target yaw angle in degrees (e.g., for 90° turns)
float kpyaw = 1.2;             // PID gains for yaw
float kdyaw = 0.1;
float kiyaw = 0.02;
float minPWM = 105.0;          // Minimum PWM to initiate rotation

// ======================= RPM Change Tracking =======================
float prevTargetRPM_Left = 0.0;
float prevTargetRPM_Right = 0.0;

// ======================= Linear Regression for Gyro Drift Compensation ===
// Estimated relationship: Gz = slope * PWM + intercept
const float slope = 0.4222;
const float intercept = -19.9526;

// --- Empirical PWM-to-RPM Regression Models for both motors ---
const float REGRESSION_SLOPE_L = 0.2957;
const float REGRESSION_INTERCEPT_L = 4.9861;

const float REGRESSION_SLOPE_R = 0.3041;
const float REGRESSION_INTERCEPT_R = 3.8851;

// ======================= PWM Limits and Deadband Compensation ============
const int MIN_PWM_OVERALL = 0;
const int MAX_PWM_OVERALL = 255;

const int MIN_EFFECTIVE_PWM_L = 85;
const int MIN_EFFECTIVE_PWM_R = 85;

// ======================= Robot Position Tracking ==========================
float currentX = 0.0;
float currentY = 0.0;

float targetX = 0.0;
float targetY = 0.0;
float targetDistance = 0.0;

float x_coord = 0.0;   // Coordinates sent via Bluetooth
float y_coord = 0.0;
bool newTargetReceived = false;  // Flag for Bluetooth waypoint update


// ======================= Motor Driver Helper Functions ====================

// Sets motor directions based on sign of PWM and applies PWM values
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

  delay(10); // Small delay for stability

  // Apply PWM with constraints
  analogWrite(ENAL, constrain(pwmL, 0, 255));
  analogWrite(ENAR, constrain(pwmR, 0, 255));
}

// Immediately stops both motors
void stopMotors() {
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
}

// Calculates gyro bias (Z-axis drift) from 200 samples
void calculateYawBias() {
  float sum = 0.0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }

  yawBias = sum / 200.0; // Average gyro drift
  Serial1.print("Yaw bias: ");
  Serial1.println(yawBias, 6);

  lastTime = millis();  // Store timestamp for integration
  Serial1.print("lastTime in calculateBias: ");
  Serial1.println(lastTime);
}

// Reads ultrasonic sensor distance in cm for given pins
long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 40000); // 40 ms timeout
  long distance = duration * 0.034 / 2.0;        // Convert to cm

  // Filter invalid data
  if (distance < MIN_VALID_DISTANCE || duration == 0) {
    return INVALID_READING;
  }

  return distance;
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void setup() {
  Wire.begin(); // Initialize I2C communication
  Serial1.begin(9600); // Start Bluetooth serial communication
  Serial1.println("Arduino: HC-05 Ready for Data Streaming!");

  // Set motor control pins as OUTPUT
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial1.println("MPU6050 init failed!");
    while (1); // Halt if MPU initialization fails
  }

  // Configure MPU6050 ranges and filtering
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Encoder pins setup with interrupts
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Ultrasonic sensor pin setup
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  stopMotors(); // Ensure motors are off at start
  
  delay(15000); // Wait before starting (possibly for setup or pairing)

  calculateYawBias(); // Calibrate initial yaw reading
  lastTime = millis(); // Record initial timestamp
  Serial1.println("Setup done.");
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

void loop() {
  unsigned long now = millis();

  // Periodic ultrasonic sensor reading
  if (now - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = now;

    long distL = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
    delay(10); // Small delay between sensor readings
    long distR = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

    // Debug print distances
    Serial.print("left distance: ");
    Serial.print(distL);
    Serial.print(" | right distance: ");
    Serial.println(distR);

    // Debounced left obstacle detection
    if (distL != INVALID_READING && distL < OBSTACLE_THRESHOLD_CM) {
      leftObstacleCounter++;
    } else {
      leftObstacleCounter = 0;
    }

    // Debounced right obstacle detection
    if (distR != INVALID_READING && distR < OBSTACLE_THRESHOLD_CM) {
      rightObstacleCounter++;
    } else {
      rightObstacleCounter = 0;
    }

    // Confirmed left obstacle
    if (leftObstacleCounter >= 2) {
      Serial1.println("LEFT OBSTACLE DETECTED");
      getPolar(currentX + 1, currentY + 1); // Placeholder behaviour
    }

    // Confirmed right obstacle
    if (rightObstacleCounter >= 2) {
      Serial1.println("RIGHT OBSTACLE DETECTED");
      getPolar(currentX + 1, currentY + 1); // Placeholder behaviour
    }
  }

  // State machine logic
  switch (currentState) {

    case IDLE:
      receiveCoordinates(); // Check for new target over Bluetooth

      if (newTargetReceived) {
        newTargetReceived = false;
        getPolar(x_coord, y_coord); // Convert to movement command
      }
      break;

    // Uncomment this block if using hardcoded waypoints instead of Bluetooth
    // case IDLE:
    //   if (currentWaypointIndex < totalWaypoints) {
    //     x_coord = waypoints[currentWaypointIndex][0];
    //     y_coord = waypoints[currentWaypointIndex][1];
    //     Serial1.print("x_coord: ");
    //     Serial1.print(x_coord);
    //     Serial1.print(" | y_coord: ");
    //     Serial1.println(y_coord);
    //     getPolar(x_coord, y_coord);
    //   } else {
    //     currentState = COMPLETED;
    //   }
    //   break;

    case ROTATING:
      controlRotation(); // Rotate to face target
      leftEncoderTicks = 0;
      rightEncoderTicks = 0;
      break;

    case MOVING_FORWARD:
      controlForward(); // Move straight towards target
      break;

    case COMPLETED:
      stopMotors();
      Serial1.println("Target reached!"); 
      currentState = IDLE;
      break;
  }
}

// Parse and receive coordinates sent over Bluetooth (Serial1)
void receiveCoordinates() {
  static String inputString = "";
  while (Serial1.available()) {
    char inChar = Serial1.read();
    if (inChar == '\n') {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex != -1) {
        x_coord = inputString.substring(0, commaIndex).toFloat();
        y_coord = inputString.substring(commaIndex + 1).toFloat();
        newTargetReceived = true;

        Serial1.print("Received from laptop: ");
        Serial1.print(x_coord);
        Serial1.print(", ");
        Serial1.println(y_coord);
      }
      inputString = ""; // Clear buffer after parsing
    } else {
      inputString += inChar; // Build up the incoming string
    }
  }
}

// Converts target (x, y) into polar form: angle + distance
void getPolar(float x, float y) {
  targetX = x;
  targetY = y;

  // Compute relative position
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;

  // Update internal position estimate
  currentX = targetX;
  currentY = targetY;
  
  // Polar conversion
  targetPos_mm = sqrt(deltaX * deltaX + deltaY * deltaY);
  targetAngle = atan2(deltaY, deltaX);

  // Debug output
  Serial1.print("New target: (");
  Serial1.print(x);
  Serial1.print(", ");
  Serial1.print(y);
  Serial1.print(") Distance: ");
  Serial1.print(targetPos_mm);
  Serial1.print("mm, Angle: ");
  Serial1.print(targetAngle * 180.0 / PI);
  Serial1.println(" degrees");

  // Begin rotation state
  currentState = ROTATING;
  delay(50); // Short wait
  calculateYawBias(); // Recalibrate yaw for new movement
}

// --- Handles rotation towards target yaw using MPU6050 and PID control ---
void controlRotation() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastLoopTime) / 1000.0; // Convert time to seconds

  lastLoopTime = currentTime;

  // --- Get IMU sensor readings ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Compensate gyroscope z-axis drift ---
  float correctedGz = g.gyro.z - yawBias;

  // --- Integrate angular velocity to estimate yaw angle ---
  yawAngle += correctedGz * dt;

  // --- Compute angular error to target in degrees ---
  float error = (targetAngle - yawAngle) * 180.0 / PI;

  // --- Normalize error to [-180, 180] degrees ---
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  // --- If within angle tolerance (~2 degrees), finish rotation ---
  if (abs(error) < 1.5) {
    stopMotors();
    Serial1.println("Rotation complete. Starting forward movement.");
    Serial1.print("Yaw (deg): ");
    Serial1.print(yawAngle * 180.0 / PI);
    Serial1.print(" | Error: ");
    Serial1.println(error);

    // Transition to forward movement
    currentState = MOVING_FORWARD;
    delay(1000); // small wait before moving forward
    return;
  }

  // --- PID Integration Term ---
  yawErrorSum += error * dt;
  yawErrorSum = constrain(yawErrorSum, -100, 100); // Avoid integral wind-up

  // --- PID Derivative Term ---
  float derivative = (error - lastYawError) / dt;
  lastYawError = error;

  // --- Compute desired angular velocity (Gz) using PID ---
  float desiredGz = kpyaw * error + kdyaw * derivative + kiyaw * yawErrorSum;
  desiredGz = constrain(desiredGz, -150, 150); // Clamp to max allowable Gz

  // --- Convert angular velocity to PWM via regression ---
  float pwm = (abs(desiredGz) - intercept) / slope;
  if (desiredGz < 0) pwm = -pwm;

  // --- Enforce minimum effective PWM threshold ---
  if (abs(pwm) < minPWM) pwm = (pwm > 0) ? minPWM : -minPWM;
  pwm = constrain(pwm, -255, 255); // Clamp PWM range

  // --- Rotate robot in place ---
  setMotorDirections(-pwm, pwm); // left and right motors in opposite direction

  // --- Debugging Output ---
  Serial1.print("Yaw (deg): ");
  Serial1.print(yawAngle * 180.0 / PI);
  Serial1.print(" | Error: ");
  Serial1.print(error);
  Serial1.print(" | Gz actual: ");
  Serial1.print(correctedGz * 180.0 / PI);
  Serial1.print(" | Gz desired: ");
  Serial1.print(desiredGz);
  Serial1.print(" | PWM: ");
  Serial1.println(pwm);
}

// --- Moves robot forward with velocity control using encoder feedback ---
void controlForward() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // --- Step 1: Copy encoder ticks safely from ISRs ---
    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    long currentRightTicks_copy = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    // --- Step 2: Convert encoder ticks to distance moved (mm) ---
    deltaRevLeft = (float)currentLeftTicks_copy / (float)PPR;
    deltaRevRight = (float)currentRightTicks_copy / (float)PPR;
    deltaDistLeft = deltaRevLeft * WHEEL_CIRCUM_MM;
    deltaDistRight = deltaRevRight * WHEEL_CIRCUM_MM;
    deltaDistance = (deltaDistLeft + deltaDistRight) / 2.0;
    currentPos_mm += deltaDistance;

    // --- Step 3: Check if destination reached ---
    if (abs(currentPos_mm - targetPos_mm) <= POSITION_TOLERANCE_MM) {
      targetRPM_Left = 0;
      targetRPM_Right = 0;
      outputPWM_Left = 0;
      outputPWM_Right = 0;
      stopMotors();

      // Send RPM snapshot to serial
      Serial1.print(currentRPM_Left);
      Serial1.print(",");
      Serial1.print(0);
      Serial1.print(",");
      Serial1.print(currentRPM_Right);
      Serial1.print(",");
      Serial1.println(0);

      // Transition to next state
      currentState = COMPLETED;
      currentWaypointIndex++;
      currentPos_mm = 0;
      return;
    }

    // --- Step 4: Outer loop PID (position -> target velocity) ---
    float posError = targetPos_mm - currentPos_mm;
    float dError_pos = (posError - prevPosError) / (deltaTime_ms / 1000.0);
    prevPosError = posError;
    float targetVelRPM = Kp_pos * posError + Kd_pos * dError_pos;

    // Clamp and quantize RPM to match motor behavior
    if (targetVelRPM < 0) targetVelRPM = 0;
    if (targetVelRPM > 150) targetVelRPM = 150;
    targetVelRPM = round(targetVelRPM / 18.75) * 18.75;
    if (targetVelRPM > 0 && targetVelRPM < MIN_MOVABLE_RPM)
      targetVelRPM = MIN_MOVABLE_RPM;
    if (abs(posError) < POSITION_TOLERANCE_MM)
      targetVelRPM = 0;

    targetRPM_Left = targetVelRPM;
    targetRPM_Right = targetVelRPM;

    // --- Step 5: Velocity PID - Left motor ---
    currentRPM_Left = deltaRevLeft / (deltaTime_ms / 60000.0); // rev/min
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

    // Enforce minimum and max PWM limits
    if (targetRPM_Left == 0) outputPWM_Left = 0;
    else if (outputPWM_Left < MIN_EFFECTIVE_PWM_L) outputPWM_Left = MIN_EFFECTIVE_PWM_L;
    if (outputPWM_Left > 255) outputPWM_Left = 255;

    // Apply direction and PWM to motor driver
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left);

    // --- Step 6: Velocity PID - Right motor ---
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

    // Enforce minimum and max PWM limits
    if (targetRPM_Right == 0) outputPWM_Right = 0;
    else if (outputPWM_Right < MIN_EFFECTIVE_PWM_R) outputPWM_Right = MIN_EFFECTIVE_PWM_R;
    if (outputPWM_Right > 255) outputPWM_Right = 255;

    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right);

    // --- Special fix: push robot if stuck near goal ---
    if (posError < 35 && posError > POSITION_TOLERANCE_MM) {
      digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
      analogWrite(ENAL, 85);

      digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
      analogWrite(ENAR, 85);
    }

    // --- Debugging output ---
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

// --- Interrupt: triggered by encoder tick on LEFT wheel ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++;
    lastPulseTimeLeft = currentTime;
  }
}

// --- Interrupt: triggered by encoder tick on RIGHT wheel ---
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}
