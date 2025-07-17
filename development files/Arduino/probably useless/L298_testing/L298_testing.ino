// --- Motor Control Pins ---
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

void setup() {
  // --- Serial Communication Setup ---
  Serial.begin(9600); // Initialize serial communication at 9600 baud
  Serial.println("--- Arduino Motor Test Start ---");

  // Set all the motor control pins to outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.println("Motor control pins set as OUTPUT.");

  // Turn off motors - Initial state
  // By setting both direction pins LOW, the motor is effectively off.
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("All motors initially set to OFF (direction pins LOW).");
  Serial.println("------------------------------------");
}

void loop() {
  Serial.println("\n--- Entering loop() iteration ---");

  // Test motor direction control
  directionControl();
  delay(1000); // Wait for 1 second

  Serial.println("--- loop() iteration complete ---");
}

// This function lets you control spinning direction of motors
void directionControl() {
  Serial.println("  -- directionControl() function called --");

  // Since jumpers are on ENA/ENB, motors are always at full speed when active.
  // We just need to set the direction pins.

  // Motor A & B spin in one direction (e.g., forward)
  Serial.println("  Setting motors FORWARD...");
  Serial.println("  Motor A: in1=HIGH, in2=LOW");
  Serial.println("  Motor B: in3=LOW, in4=HIGH (Note: this is usually FORWARD for Motor B if connected similarly to Motor A)");
  digitalWrite(in1, HIGH); // Motor A forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); // Motor B forward
  digitalWrite(in4, HIGH);
  delay(2000); // Run for 2 seconds
  Serial.println("  Motors ran FORWARD for 2 seconds.");

  // Now change motor directions (e.g., backward)
  Serial.println("  Setting motors BACKWARD...");
  Serial.println("  Motor A: in1=LOW, in2=HIGH");
  Serial.println("  Motor B: in3=HIGH, in4=LOW");
  digitalWrite(in1, LOW); // Motor A backward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); // Motor B backward
  digitalWrite(in4, LOW);
  delay(2000); // Run for 2 seconds
  Serial.println("  Motors ran BACKWARD for 2 seconds.");

  // // Turn off motors by setting both direction pins LOW
  // Serial.println("  Turning motors OFF...");
  // digitalWrite(in1, LOW);
  // digitalWrite(in2, LOW);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, LOW);
  // Serial.println("  Motors are now OFF.");
  Serial.println("  -- directionControl() function finished --");
}

// This function demonstrates motor on/off.
// Note: With ENA/ENB jumpers on, actual speed control via analogWrite is not possible.
// Motors will run at full speed when their direction pins are set.
void speedControl() {
  Serial.println("  -- speedControl() function called --");

  // Turn on motors (they will run at full speed due to jumpers)
  Serial.println("  Turning motors ON (full speed, FORWARD)...");
  Serial.println("  Motor A: in1=HIGH, in2=LOW");
  Serial.println("  Motor B: in3=HIGH, in4=LOW"); // Note: You had these as HIGH, LOW for B forward here.
                                                // Consistent with previous, it was LOW, HIGH. Let's make it consistent.
  digitalWrite(in1, HIGH); // Motor A direction (e.g., forward)
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // Motor B direction (e.g., forward, matching 'directionControl' above)
  digitalWrite(in4, LOW);
  delay(3000); // Run at full speed for 3 seconds
  Serial.println("  Motors ran at full speed (FORWARD) for 3 seconds.");

  // Now turn off motors
  Serial.println("  Turning motors OFF...");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(1000); // Stay off for 1 second
  Serial.println("  Motors are now OFF for 1 second.");
  Serial.println("  -- speedControl() function finished --");
}