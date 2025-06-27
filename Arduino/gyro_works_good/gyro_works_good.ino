#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float yawAngle = 0.0;
float yawBias = 0.05;
const int biasSamples = 200;
unsigned long lastUpdate = 0;
unsigned long lastBiasUpdate = 0;
const unsigned long biasInterval = 5000;

const float MAX_BIAS_CHANGE = 0.01;  // Don't allow sudden big changes in bias

#define STATIONARY_WINDOW 20
float recentGz[STATIONARY_WINDOW];
int gzIndex = 0;

// Store EMA instead of raw overwrite
float emaBias = 0.05;
const float emaAlpha = 0.05;

bool isStationary() {
  for (int i = 0; i < STATIONARY_WINDOW; i++) {
    if (abs(recentGz[i]) > 0.02) return false;
  }
  return true;
}

float calculateYawBias() {
  float sum = 0.0;
  sensors_event_t a, g, temp;

  for (int i = 0; i < biasSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sum += g.gyro.z;
    delay(5);
  }

  return sum / biasSamples;
}

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  yawBias = calculateYawBias();
  emaBias = yawBias;
  lastUpdate = millis();
  lastBiasUpdate = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;
  lastUpdate = now;

  float gz = g.gyro.z;

  // Update Gz buffer
  recentGz[gzIndex] = gz;
  gzIndex = (gzIndex + 1) % STATIONARY_WINDOW;

  // Bias recalculation condition
  if (isStationary() && (now - lastBiasUpdate >= biasInterval)) {
    float newBias = calculateYawBias();

    // Only accept new bias if it's not way off
    if (abs(newBias - yawBias) < MAX_BIAS_CHANGE) {
      emaBias = emaAlpha * newBias + (1 - emaAlpha) * emaBias;
      yawBias = emaBias;
      Serial.print("Updated bias: ");
      Serial.println(yawBias, 6);
    } else {
      Serial.println("Bias jump too large. Ignored.");
    }

    lastBiasUpdate = now;
  }

  float correctedGz = gz - yawBias;
  yawAngle += correctedGz * dt;

  Serial.print("Yaw Angle (deg): ");
  Serial.println(yawAngle * 180.0 / PI);

  delay(20);
}
