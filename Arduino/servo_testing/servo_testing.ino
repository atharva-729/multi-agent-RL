#include <Servo.h>

Servo myServo;  // Create a Servo object to control the servo motor
int servoPin = 9; // Define the pin where the servo is connected

void setup() {
  // Attach the Servo object to the servo pin.
  // This tells the Arduino which pin will be used to control the servo.
  myServo.attach(servoPin);
}

void loop() {
  // Sweep the servo from 0 to 180 degrees.
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(angle);     // Send the current angle to the servo
    delay(15);              // Small delay to allow the servo to reach the position
  }

  // Sweep the servo from 180 to 0 degrees.
  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(angle);     // Send the current angle to the servo
    delay(15);              // Small delay to allow the servo to reach the position
  }
}