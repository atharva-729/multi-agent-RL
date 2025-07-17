#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float yawAngle = 0;
float yawBias = 0;
unsigned long lastTime = 0;

void setup() {
  Serial1.begin(9600);
  while (!Serial1)
    delay(10);

  if (!mpu.begin()) {
    Serial1.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial1.println("MPU6050 Found!");

  // Reset MPU6050 (only works on some variants)
  mpu.reset();  // Try this. If it causes issues, remove it.

  delay(1000); // Let it settle

  calculateYawBias(); // Do this when robot is still

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float correctedGz = g.gyro.z - yawBias;
  yawAngle += correctedGz * dt;

  // Convert to degrees
  float yawDeg = yawAngle * 180.0 / PI;
  float GzDeg = correctedGz * 180.0 / PI;

  Serial1.print("Yaw (deg): ");
  Serial1.print(yawDeg);
  Serial1.print(" | Gz (deg/s): ");
  Serial1.print(GzDeg);
  Serial1.print(" | Raw Gz: ");
  Serial1.print(g.gyro.z, 6);
  Serial1.print(" | Temp: ");
  Serial1.println(temp.temperature);

  delay(200); // Adjust based on how fast you want updates
}

void calculateYawBias() {
  float sum = 0.0;
  sensors_event_t a, g, temp;
  Serial1.println("Calculating yaw bias...");
  for (int i = 0; i < 200; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }
  yawBias = sum / 200.0;
  Serial1.print("Yaw bias: ");
  Serial1.println(yawBias, 6);
}
