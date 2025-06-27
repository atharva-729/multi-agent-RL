#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Motor Driver Pins ---
const int ENAL = 9;
const int IN1L = 8;
const int IN2L = 7;
const int ENAR = 10;
const int IN3R = 5;
const int IN4R = 4;

// --- PWM Sweep Settings ---
const int PWM_START = 0;
const int PWM_END = 255;
const int PWM_STEP = 5;
const unsigned long STABILIZE_TIME_MS = 500;
const unsigned long MEASURE_TIME_MS = 300;

// --- MPU6050 ---
Adafruit_MPU6050 mpu;

void setup() {
  Serial1.begin(9600);
  delay(1000);

  // --- Motor Setup ---
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);
  stopMotors();

  // --- MPU6050 Init ---
  Wire.begin();
  if (!mpu.begin()) {
    Serial1.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial1.println("PWM,Gz_dps");
  delay(15000); // Wait for user to open Serial Monitor / HC-05 connection
}

void loop() {
  for (int pwm = PWM_START; pwm <= PWM_END; pwm += PWM_STEP) {
    rotateInPlace(pwm);  // Rotate in place

    delay(STABILIZE_TIME_MS);

    float gz_sum = 0;
    int samples = 0;
    unsigned long startTime = millis();
    while (millis() - startTime < MEASURE_TIME_MS) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gz_sum += g.gyro.z * 57.2958; // rad/s to deg/s
      samples++;
      delay(5);
    }

    stopMotors();
    delay(100);  // Let the motors stop completely

    float gz_avg = gz_sum / samples;
    Serial1.print(pwm); Serial1.print(","); Serial1.println(gz_avg);
    delay(200);
  }

  Serial1.println("Sweep complete");
  while (true); // halt
}

void rotateInPlace(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, HIGH);
  analogWrite(ENAL, pwm);

  digitalWrite(IN3R, HIGH);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, pwm);
}

void stopMotors() {
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
}
