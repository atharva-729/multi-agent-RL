// Define L298N connections for the Left Side Motors (Motor A)
const int ENAL = 9;  // Enable Pin for Left Motors (PWM)
const int IN1L = 8;  // Input 1 for Left Motors
const int IN2L = 7;  // Input 2 for Left Motors

// Define L298N connections for the Right Side Motors (Motor B)
const int ENAR = 10; // Enable Pin for Right Motors (PWM)
const int IN3R = 5;  // Input 3 for Right Motors
const int IN4R = 4;  // Input 4 for Right Motors

// Delay settings
const int sweepDelay = 200; // Delay between PWM steps (in milliseconds)
const int stopDelay = 1000; // Delay when motors are stopped (in milliseconds)

void setup() {
  // Set all control pins as OUTPUT
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Initialize all motors to stop
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0); // PWM 0 to stop

  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0); // PWM 0 to stop

  Serial.begin(9600); // Start serial communication for debugging
  Serial.println("L298N Motor Driver Test Started.");
  Serial.println("Ensure motors are free to spin or robot is safely elevated.");
}

void loop() {
  // --- Test Left Motors (Forward) ---
  Serial.println("\nTesting Left Motors: FORWARD");
  digitalWrite(IN1L, HIGH); // Set direction forward
  digitalWrite(IN2L, LOW);

  for (int pwmValue = 60; pwmValue <= 255; pwmValue += 5) { // Sweep PWM from 0 to 255
    analogWrite(ENAL, pwmValue); // Apply PWM
    Serial.print("Left Forward PWM: ");
    Serial.println(pwmValue);
    delay(sweepDelay);
  }
  delay(stopDelay); // Wait at full speed

  // Stop Left Motors
  Serial.println("Stopping Left Motors.");
  analogWrite(ENAL, 0); // Set PWM to 0
  delay(stopDelay);

  // --- Test Left Motors (Backward) ---
  Serial.println("Testing Left Motors: BACKWARD");
  digitalWrite(IN1L, LOW); // Set direction backward
  digitalWrite(IN2L, HIGH);

  for (int pwmValue = 0; pwmValue <= 255; pwmValue += 5) { // Sweep PWM from 0 to 255
    analogWrite(ENAL, pwmValue); // Apply PWM
    Serial.print("Left Backward PWM: ");
    Serial.println(pwmValue);
    delay(sweepDelay);
  }
  delay(stopDelay); // Wait at full speed

  // Stop Left Motors
  Serial.println("Stopping Left Motors.");
  digitalWrite(IN1L, LOW); // Reset direction pins to LOW for a clean stop
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0);
  delay(stopDelay * 2); // Longer stop before next motor

  // --- Test Right Motors (Forward) ---
  Serial.println("Testing Right Motors: FORWARD");
  digitalWrite(IN3R, HIGH); // Set direction forward
  digitalWrite(IN4R, LOW);

  for (int pwmValue = 0; pwmValue <= 255; pwmValue += 5) { // Sweep PWM from 0 to 255
    analogWrite(ENAR, pwmValue); // Apply PWM
    Serial.print("Right Forward PWM: ");
    Serial.println(pwmValue);
    delay(sweepDelay);
  }
  delay(stopDelay); // Wait at full speed

  // Stop Right Motors
  Serial.println("Stopping Right Motors.");
  analogWrite(ENAR, 0);
  delay(stopDelay);

  // --- Test Right Motors (Backward) ---
  Serial.println("Testing Right Motors: BACKWARD");
  digitalWrite(IN3R, LOW); // Set direction backward
  digitalWrite(IN4R, HIGH);

  for (int pwmValue = 0; pwmValue <= 255; pwmValue += 5) { // Sweep PWM from 0 to 255
    analogWrite(ENAR, pwmValue); // Apply PWM
    Serial.print("Right Backward PWM: ");
    Serial.println(pwmValue);
    delay(sweepDelay);
  }
  delay(stopDelay); // Wait at full speed

  // Stop Right Motors
  Serial.println("Stopping Right Motors.");
  digitalWrite(IN3R, LOW); // Reset direction pins to LOW for a clean stop
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
  delay(stopDelay * 3); // Longer stop before repeating the whole sequence
}