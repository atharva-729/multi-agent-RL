#include <Arduino.h> // Core Arduino functions
#include <Wire.h>    // I2C communication
#include <Adafruit_MPU6050.h> // MPU6050 sensor library
#include <Adafruit_Sensor.h>  // Sensor base class

// --- Thresholds and constants ---
#define OBSTACLE_THRESHOLD_CM 30
#define MIN_VALID_DISTANCE 5
#define INVALID_READING -1

unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 500; // sensor polling interval (not used actively)

int leftObstacleCounter = 0;
int rightObstacleCounter = 0;

Adafruit_MPU6050 mpu; // MPU6050 IMU object

// --- Robot states for navigation ---
enum RobotState {
  IDLE,
  ROTATING,
  MOVING_FORWARD,
  COMPLETED
};

RobotState currentState = IDLE; // initial state

// --- Hardcoded waypoints (in mm) ---
float waypoints[][2] = {
  {450.0, 0.0},
  {450.0, 450.0},
  {0.0, 450.0},
  {0.0, 0.0}
};

const int totalWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);
int currentWaypointIndex = 0; // index of current target waypoint

// --- Motor Driver (L298N) Pin Assignments ---
const int ENAL = 10;
const int IN1L = 5;
const int IN2L = 4;

const int ENAR = 9;
const int IN3R = 8;
const int IN4R = 7;

// --- Orientation & Position Tracking ---
float yawAngle = 0.0;
float yawBias = 0.0;
unsigned long lastTime = 0;

float targetPos_mm = 0.0;
float targetAngle = 0.0;

float currentX = 0.0;
float currentY = 0.0;

float targetX = 0.0;
float targetY = 0.0;

float x_coord = 0.0;   // Coordinates sent via Bluetooth
float y_coord = 0.0;
bool newTargetReceived = false;  // Flag for Bluetooth waypoint update


// --- Helper: Sets motor direction and PWM ---
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

  delay(10); // short delay before PWM
  analogWrite(ENAL, constrain(pwmL, 0, 255));
  analogWrite(ENAR, constrain(pwmR, 0, 255));
}

// --- Stop all motors ---
void stopMotors() {
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
}

// --- Compute initial yaw bias from stationary MPU readings ---
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

  lastTime = millis(); // for drift correction in future (unused here)
}

// --- Ultrasonic distance measurement (not used) ---
long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 40000); // 40ms timeout
  long distance = duration * 0.034 / 2.0;

  if (distance < MIN_VALID_DISTANCE || duration == 0) {
    return INVALID_READING;
  }

  return distance;
}

// --- Arduino Setup ---
void setup() {
  Wire.begin();
  Serial1.begin(9600); // For Bluetooth via HC-05
  Serial1.println("Arduino: HC-05 Ready for Data Streaming!");

  // Motor pin setup
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial1.println("MPU6050 init failed!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  stopMotors(); // ensure robot starts stationary
  delay(5000);  // wait before starting to allow placement
  calculateYawBias(); // get initial gyro offset

  lastTime = millis();
  Serial1.println("Setup done.");
}

// --- Main Loop: State machine for waypoint navigation ---
void loop() {
  unsigned long now = millis();

  switch (currentState) {
    case IDLE:
      if (currentWaypointIndex < totalWaypoints) {
        // Get next target coordinates
        x_coord = waypoints[currentWaypointIndex][0];
        y_coord = waypoints[currentWaypointIndex][1];

        Serial1.print("x_coord: ");
        Serial1.print(x_coord);
        Serial1.print(" | y_coord: ");
        Serial1.println(y_coord);

        getPolar(x_coord, y_coord); // compute angle and distance
      } else {
        currentState = COMPLETED;
      }
      break;

    case ROTATING:
      controlRotation(); // turn toward target
      break;

    case MOVING_FORWARD:
      controlForward();  // move toward target
      break;

    case COMPLETED:
      stopMotors();
      currentState = IDLE; // move to next waypoint
      break;
  }
}

// --- Parses incoming coordinates via Bluetooth (unused here) ---
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
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

// --- Converts target (x, y) into polar (angle, distance) ---
void getPolar(float x, float y) {
  targetX = x;
  targetY = y;

  float deltaX = targetX - currentX;
  float deltaY = targetY - currentY;

  currentX = targetX;
  currentY = targetY;

  targetPos_mm = sqrt(deltaX * deltaX + deltaY * deltaY);
  targetAngle = atan2(deltaY, deltaX);

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

// --- Open-loop rotation: estimate time using fixed angular speed ---
void controlRotation() {
  float targetAngleDeg = targetAngle * 180.0 / PI;
  float rotationDuration = abs(targetAngleDeg) / 76.0 * 1000.0; // empirical rotation rate: 76 deg/sec

  int pwm = 150;
  if (targetAngleDeg < 0) {
    setMotorDirections(pwm, -pwm); // clockwise
  } else {
    setMotorDirections(-pwm, pwm); // counter-clockwise
  }

  Serial1.print("Rotating ");
  Serial1.print(targetAngleDeg);
  Serial1.print(" degrees. Estimated time: ");
  Serial1.print(rotationDuration);
  Serial1.println(" ms");

  delay((unsigned long)rotationDuration); // open-loop wait
  stopMotors();

  Serial1.println("Rotation complete. Starting forward movement.");
  currentState = MOVING_FORWARD;
  delay(500); // short pause before moving
}

// --- Open-loop forward movement: estimate time using speed ---
void controlForward() {
  float targetDistance_m = targetPos_mm / 1000.0;
  float forwardDuration = targetDistance_m / 0.32 * 1000.0; // speed = 0.32 m/s

  int pwm = 100;

  // Both motors forward
  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
  analogWrite(ENAL, pwm);

  digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
  analogWrite(ENAR, pwm);

  Serial1.print("Moving forward ");
  Serial1.print(targetDistance_m);
  Serial1.print(" m. Estimated time: ");
  Serial1.print(forwardDuration);
  Serial1.println(" ms");

  delay((unsigned long)forwardDuration); // open-loop wait
  stopMotors();

  Serial1.println("Forward movement complete.");
  currentState = COMPLETED;
  currentWaypointIndex++;
}
