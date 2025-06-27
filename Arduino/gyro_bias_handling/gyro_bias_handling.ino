#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float yawAngle = 0.0;          // in radians
float yawBias = 0.05;          // fallback default in rad/s
unsigned long lastTime = 0;

const int biasSamples = 200;   // number of samples for bias calculation
bool biasCalculated = false;

// Stub: Replace this with your own encoder-based check
bool isRobotStationary() {
  // Implement actual check based on encoder speed
  return true;
}

void calculateYawBias() {
  float sum = 0.0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < biasSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5); // total time ~1s
  }

  yawBias = sum / biasSamples;
  Serial.print("Bias recalculated: ");
  Serial.println(yawBias, 6);
}

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    while (1) delay(10);
  }

  Serial.println("MPU6050 Found and Initialized!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  Serial.println("Calculating initial yaw bias...");
  calculateYawBias();
  biasCalculated = true;

  lastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // seconds
  lastTime = currentTime;

  // Subtract bias
  float correctedGz = g.gyro.z - yawBias;

  // Integrate to get yaw angle
  yawAngle += correctedGz * dt;

  // Convert to degrees for readability
  float yawDeg = yawAngle * 180.0 / PI;
  Serial.print("Yaw Angle (deg): ");
  Serial.println(yawDeg);

  // Recalculate bias if robot is stationary
  if (isRobotStationary()) {
    calculateYawBias();
  }

  delay(50); // 1 Hz sampling
}
