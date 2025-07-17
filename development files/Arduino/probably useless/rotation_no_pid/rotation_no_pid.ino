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

float targetAngle = 90.0 * (PI / 180.0);  // rotate 90 degrees
