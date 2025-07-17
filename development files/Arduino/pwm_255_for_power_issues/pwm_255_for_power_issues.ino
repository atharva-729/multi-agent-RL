#include <Arduino.h> // Standard Arduino library

// --- Bluetooth Module Pins (HC-05 using Hardware Serial1 on Mega) ---
// On Arduino Mega:
// RX1 (Pin 19) will receive data from HC-05 TX
// TX1 (Pin 18) will send data to HC-05 RX (via voltage divider)
// Ensure your HC-05's baud rate is set to 9600 (default) or match it here.

// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 10;  // PWM pin for Left Motor speed (Connect to L298N ENB)
const int IN1L = 5;  // Left Motor Direction Pin 1 (Connect to L298N IN3)
const int IN2L = 4;  // Left Motor Direction Pin 2 (Connect to L298N IN4)

// Right Motor pins
const int ENAR = 9; // PWM pin for Right Motor speed (Connect to L298N ENA)
const int IN3R = 8;  // Right Motor Direction Pin 1 (Connect to L298N IN1)
const int IN4R = 7;  // Right Motor Direction Pin 2 (Connect to L298N IN2)

void setup() {
    // Motor Driver Pin Setup for LEFT Motor
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Motor Driver Pin Setup for RIGHT Motor
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
  delay(3000);
}

void loop() {
  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW); analogWrite(ENAL, 100);
  digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW); analogWrite(ENAR, 100);
  
}