#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float yawAngle = 0.0;              // in radians
const float yawBias = 0.05;        // rad/s, hardcoded
unsigned long prevTime = 0;
unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();                    // Uses Mega's default I2C pins: SDA=20, SCL=21
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected.");
  prevTime = millis();
  lastPrint = millis();
}

void loop() {
  // Current time
  unsigned long currTime = millis();
  float dt = (currTime - prevTime) / 1000.0; // Convert ms to seconds
  prevTime = currTime;

  // Read raw gyro values
  int16_t gx, gy, gz;
  mpu.getRotation(&gx, &gy, &gz);

  // Convert to rad/s (131 LSB/deg/s → multiply by π/180 to get rad/s)
  float yawRate = ((float)gz / 131.0) * (3.1415926 / 180.0); 

  // Subtract hardcoded bias
  float correctedYawRate = yawRate - yawBias;

  // Integrate yaw rate over time
  yawAngle += correctedYawRate * dt;

  // Print yaw angle once per second
  if (currTime - lastPrint >= 100) {
    Serial.print("Yaw Angle (deg): ");
    Serial.println(yawAngle * (180.0 / 3.1415926)); // convert to degrees for easier understanding
    lastPrint = currTime;
  }
}
