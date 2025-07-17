#include <Arduino.h> // Standard Arduino library
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define OBSTACLE_THRESHOLD_CM 30
#define MIN_VALID_DISTANCE 5
#define INVALID_READING -1

unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 500; // ms

int leftObstacleCounter = 0;
int rightObstacleCounter = 0;

Adafruit_MPU6050 mpu;

enum RobotState {
  IDLE,
  ROTATING,
  MOVING_FORWARD,
  COMPLETED
};

RobotState currentState = IDLE;

float waypoints[][2] = {
  {450.0, 0.0},
  {450.0, 450.0},
  {0.0, 450.0},
  {0.0, 0.0} 
};

const int totalWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);
int currentWaypointIndex = 0;

// L298N Motor Driver Pins
const int ENAL = 10;
const int IN1L = 5;
const int IN2L = 4;

const int ENAR = 9;
const int IN3R = 8;
const int IN4R = 7;

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

float yawAngle = 0.0;
float yawBias = 0.0;
unsigned long lastTime = 0;
float lastYawError = 0.0;
float yawErrorSum = 0.0;
float lastYaw = 0.0;

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
float Kp_L = 0.9;  // Proportional gain for Left Motor (adjust as needed)
float Ki_L = 0.0;  // Integral gain for Left Motor (start with 0.0, tune later)
float Kd_L = 0.15;  // Derivative gain for Left Motor (start with 0.0, tune later)

float targetRPM_Left = 56.25; // Desired target RPM for Left Motor
float currentRPM_Left = 0.0;
float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;
float outputPWM_Left = 0.0; // This variable will be cumulatively updated

// --- PID Control Variables for RIGHT Motor ---
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

// --- Position Control Parameters ---
const float WHEEL_CIRCUM_MM = 219.9; // 70 mm dia
float currentPos_mm = 0.0;
float targetPos_mm = 0.0; // <-- You can change this later via Bluetooth if desired
float posError = 0.0;
float prevPosError = 0.0;

// gains for position control
float Kp_pos = 0.25;  // You will tune this
float Kd_pos = 0.05;  // Optional
float Ki_pos = 0.0;  // really not needed

// gains for rotation control
float targetAngle = 0.0; //45.0 * (PI / 180.0);  // rotate 90 degrees
float kpyaw = 1.2;        // proportional gain
float kdyaw = 0.1;
float kiyaw = 0.02;
float minPWM = 105.0;  // minimum PWM to overcome static friction

float prevTargetRPM_Left = 0.0;
float prevTargetRPM_Right = 0.0;

// Linear regression model: Gz = slope * PWM + intercept
const float slope = 0.4222;
const float intercept = -19.9526;

// --- Motor Characterization (Regression Lines) ---
const float REGRESSION_SLOPE_L = 0.2957;
const float REGRESSION_INTERCEPT_L = 4.9861;

const float REGRESSION_SLOPE_R = 0.3041;
const float REGRESSION_INTERCEPT_R = 3.8851;

// --- PWM Output Limits ---
const int MIN_PWM_OVERALL = 0; // Overall minimum PWM (0 for off)
const int MAX_PWM_OVERALL = 255; // Overall maximum PWM

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
const int MIN_EFFECTIVE_PWM_L = 80; // For Left Motor (adjust if needed for 9V)
const int MIN_EFFECTIVE_PWM_R = 80; // For Right Motor (adjust if needed for 9V)

// --- Position and Orientation ---
float currentX = 0.0;
float currentY = 0.0;

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

  lastTime = millis();
  Serial1.print("lastTime in calculateBias: ");
  Serial1.println(lastTime);
}

long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 40000); // 40 ms timeout
  long distance = duration * 0.034 / 2.0;

  if (distance < MIN_VALID_DISTANCE || duration == 0) {
    return INVALID_READING;
  }

  return distance;
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
  // mpu.reset();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Initialize both motors to stop
  stopMotors();
  
  delay(5000);

  calculateYawBias();
  lastTime = millis();
  Serial1.println("Setup done.");
}


///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
float x_coord = 0.0;
float y_coord = 0.0;
bool newTargetReceived = false;


void loop() {

  // if (Serial1.available()) {
  //   String input = Serial1.readStringUntil('\n');
  //   input.trim();
    
  //   int commaIndex = input.indexOf(',');
  //   if (commaIndex > 0) {
  //     x_coord = input.substring(0, commaIndex).toFloat();
  //     y_coord = input.substring(commaIndex + 1).toFloat();
  //     currentState = IDLE; // remove this after demo
  //    }
  // delay(100);
  // }

  unsigned long now = millis();

  if (now - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = now;

    long distL = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
    delay(10); // wait before pinging next sensor
    long distR = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("left distance: ");
    Serial.print(distL);
    Serial.print(" | right distance: ");
    Serial.println(distR);

    // Check left
    if (distL != INVALID_READING && distL < OBSTACLE_THRESHOLD_CM) {
      leftObstacleCounter++;
    } else {
      leftObstacleCounter = 0;
    }

    // Check right
    if (distR != INVALID_READING && distR < OBSTACLE_THRESHOLD_CM) {
      rightObstacleCounter++;
    } else {
      rightObstacleCounter = 0;
    }

    // Debounced obstacle detection
    if (leftObstacleCounter >= 2) {
      Serial1.println("LEFT OBSTACLE DETECTED");
      getPolar(currentX + 1, currentY + 1);
    }

    if (rightObstacleCounter >= 2) {
      Serial1.println("RIGHT OBSTACLE DETECTED");
      getPolar(currentX + 1, currentY + 1);
    }
  }

  // if (!(x_coord == 0.0 && y_coord == 0.0)) {
    // State machine for 2D navigation
    switch (currentState) {
      case IDLE:
      // receiveCoordinates();

      // if (newTargetReceived) {
      //   newTargetReceived = false;
      //   getPolar(x_coord, y_coord);
      // }
      // break;

      // this was for hardcoded coordinates
        if (currentWaypointIndex < totalWaypoints) {
          // Set new target
          x_coord = waypoints[currentWaypointIndex][0];
          y_coord = waypoints[currentWaypointIndex][1];

          Serial1.print("x_coord: ");
          Serial1.print(x_coord);
          Serial1.print(" | y_coord: ");
          Serial1.println(y_coord);

          // resetGyro();   // optional: reset yaw tracking
          // resetEncoders();
          getPolar(x_coord, y_coord);

        } else {
          currentState = COMPLETED;
        }
        break;
        //*/

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
        // Serial1.println("Target reached!"); 
        currentState = IDLE;
        break;
    }
  // }

}

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
      inputString = ""; // clear after parsing
    } else {
      inputString += inChar;
    }
  }
}


void getPolar(float x, float y) {
  targetX = x;
  targetY = y;

    // Calculate polar coordinates
  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;

  currentX = targetX;
  currentY = targetY;

  // yawAngle = targetAngle;  // to tackle drift when going forward
  
  targetPos_mm = sqrt(deltaX * deltaX + deltaY * deltaY);
  targetAngle = atan2(deltaY, deltaX);
  // targetAngle -= lastYaw;
  
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
}

void controlRotation() {
  // Convert target angle to time needed (degrees / (degrees/sec))
  float targetAngleDeg = targetAngle * 180.0 / PI;
  float rotationDuration = abs(targetAngleDeg) / 76.0 * 1000.0; // in ms

  int pwm = 150;
  if (targetAngleDeg < 0) {
    // rotate clockwise
    setMotorDirections(pwm, -pwm);
  } else {
    // rotate counter-clockwise
    setMotorDirections(-pwm, pwm);
  }

  Serial1.print("Rotating ");
  Serial1.print(targetAngleDeg);
  Serial1.print(" degrees. Estimated time: ");
  Serial1.print(rotationDuration);
  Serial1.println(" ms");

  delay((unsigned long)rotationDuration);

  stopMotors();
  Serial1.println("Rotation complete. Starting forward movement.");

  currentState = MOVING_FORWARD;
  delay(500);  // Small buffer
}

void controlForward() {
  float targetDistance_m = targetPos_mm / 1000.0;
  float forwardDuration = targetDistance_m / 0.32 * 1000.0; // in ms

  int pwm = 100;

  // Set motor directions forward
  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
  analogWrite(ENAL, pwm);

  digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
  analogWrite(ENAR, pwm);

  Serial1.print("Moving forward ");
  Serial1.print(targetDistance_m);
  Serial1.print(" m. Estimated time: ");
  Serial1.print(forwardDuration);
  Serial1.println(" ms");

  delay((unsigned long)forwardDuration);

  stopMotors();
  Serial1.println("Forward movement complete.");

  currentState = COMPLETED;
  currentWaypointIndex++;
}
