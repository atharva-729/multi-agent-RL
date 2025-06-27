#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// L298N Motor Driver Pins
const int ENAL = 9;
const int IN1L = 8;
const int IN2L = 7;

const int ENAR = 10;
const int IN3R = 5;
const int IN4R = 4;

float yawAngle = 0.0;
float yawBias = 0.0;
unsigned long lastTime = 0;

float targetAngle = 45.0 * (PI / 180.0);  // rotate 90 degrees
float kp = 150;        // proportional gain
float minPWM = 155.0;  // minimum PWM to overcome static friction

// Linear regression model: Gz = slope * PWM + intercept
const float slope = 0.9368;
const float intercept = -126.8446;

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

void setup() {
  Wire.begin();
  Serial1.begin(9600);

  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  if (!mpu.begin()) {
    Serial1.println("MPU6050 init failed!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(10000);

  calculateYawBias();
  lastTime = millis();
  Serial1.println("Setup done. Starting rotation...");
}

void loop() {
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

  if (abs(error) < 2.0 * PI / 180.0) {  // ~2 degree tolerance
    stopMotors();
    Serial1.println("Target reached. Stopping.");
    Serial1.print("Yaw (deg): ");
    Serial1.print(yawAngle * 180.0 / PI);
    Serial1.print(" | Error: ");
    Serial1.println(error * 180.0 / PI);
    while (true);
  }

  // Convert current angular error to deg/s desired yaw rate
  float desiredGz = kp * error;  // deg/s because correctedGz is in rad/s

  // Cap desiredGz to a reasonable max (you can tune this)
  desiredGz = constrain(desiredGz, -150, 150);

  // Compute base PWM from linear regression
  float pwm = (desiredGz - intercept) / slope;

  // Apply minimum PWM constraint
  if (abs(pwm) < minPWM) {
    pwm = (pwm > 0) ? minPWM : -minPWM;
  }

  pwm = constrain(pwm, -255, 255);
  setMotorDirections(-pwm, pwm);  // rotate in place

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
