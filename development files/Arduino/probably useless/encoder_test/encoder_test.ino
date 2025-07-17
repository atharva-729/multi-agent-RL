// --- Motor Control Pins (from your existing motor code) ---
// Motor A connections
// enA is the Enable pin for Motor A. With the jumper on, it's always HIGH (enabled).
// So, we don't need to control it from the Arduino for basic operation.
int in1 = 8; // Input 1 for Motor A direction
int in2 = 7; // Input 2 for Motor A direction

// Motor B connections
// enB is the Enable pin for Motor B. With the jumper on, it's always HIGH (enabled).
// So, we don't need to control it from the Arduino for basic operation.
int in3 = 5; // Input 1 for Motor B direction
int in4 = 4; // Input 2 for Motor B direction

// --- TCRT5000 Sensor Pin (from your existing sensor code) ---
// Connect the DIGITAL output (D0) of TCRT5000 module to Arduino's Digital Pin 2.
const int TCRT5000_PIN = 3; // Using Digital Pin 2 as specified

// --- Variables for Motor Timing ---
// This will help us control how long the motor runs in one direction
unsigned long motorRunStartTime = 0; // Stores the time when motor started
const long motorRunDuration = 5000;  // Run motors for 5 seconds (5000 milliseconds)

// --- State variable for motor direction ---
// 0 = Forward, 1 = Backward, 2 = Stopped
int motorDirectionState = 0;

void setup() {
  // --- Motor Setup ---
  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  // By setting both direction pins LOW, the motor is effectively off.
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // --- Sensor Setup ---
  // Set the TCRT5000 sensor pin as an INPUT
  pinMode(TCRT5000_PIN, INPUT);

  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.println("Robot Test: Motors Running & TCRT5000 Digital Reading");
  Serial.println("--------------------------------------------------");

  // Record the start time for the first motor run
  motorRunStartTime = millis();
}

void loop() {
  // --- Motor Control Logic ---
  // Check if it's time to change motor direction or stop
  if (millis() - motorRunStartTime >= motorRunDuration) {
    // Time to change state!

    // Transition to the next state
    motorDirectionState++;
    if (motorDirectionState > 1) { // We have 0 (Forward), 1 (Backward), so reset to 0 after 1
      motorDirectionState = 0;
    }

    // Update motor direction based on the new state
    if (motorDirectionState == 0) {
      // Motors Forward
      Serial.println("Motors: FORWARD");
      digitalWrite(in1, HIGH); // Motor A forward
      digitalWrite(in2, LOW);
      digitalWrite(in3, LOW);  // Motor B forward
      digitalWrite(in4, HIGH);
    } else if (motorDirectionState == 1) {
      // Motors Backward
      Serial.println("Motors: BACKWARD");
      digitalWrite(in1, LOW);  // Motor A backward
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH); // Motor B backward
      digitalWrite(in4, LOW);
    }

    // Reset the timer for the new state
    motorRunStartTime = millis();
  }

  // --- TCRT5000 Digital Sensor Reading ---
  // This part runs continuously in the loop, regardless of motor state changes.
  int sensorValue = digitalRead(TCRT5000_PIN); // Read the digital value (HIGH or LOW)

  // Print the sensor value to the Serial Monitor:
  Serial.print("Sensor Value (Digital): ");
  Serial.println(sensorValue); // Will print 0 or 1

  // Small delay to make sensor readings readable without overwhelming serial
  delay(50); // Read every 50 milliseconds
}