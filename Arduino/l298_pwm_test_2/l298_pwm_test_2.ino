// Define L298N connections for the Left Side Motors (Motor A)
const int ENAL = 9;  // Enable Pin for Left Motors (PWM)
const int IN1L = 8;  // Input 1 for Left Motors
const int IN2L = 7;  // Input 2 for Left Motors

// Define L298N connections for the Right Side Motors (Motor B)
const int ENAR = 10; // Enable Pin for Right Motors (PWM)
const int IN3R = 5;  // Input 3 for Right Motors
const int IN4R = 4;  // Input 4 for Right Motors

// Test specific delay settings
const int sweepDelay = 200; // Delay between PWM steps (in milliseconds)
const int stopDelay = 2000; // Delay when motors are stopped (2 seconds)

// --- Custom Test Parameters ---
const int startPWM = 100; // Start PWM from a higher value (e.g., 100 or 150 or 200)
const int pwmIncrement = 10; // How much to increase PWM each step
const int endPWM = 255;     // Max PWM value

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
  Serial.println("L298N Motor Driver PWM Threshold Test Started.");
  Serial.println("Focus on the *lowest* PWM value at which motors start moving.");
}

void loop() {
  // --- Test Left Motors (Forward) ---
  Serial.println("\n--- Testing Left Motors: FORWARD (PWM from " + String(startPWM) + " to " + String(endPWM) + ") ---");
  digitalWrite(IN1L, HIGH); // Set direction forward
  digitalWrite(IN2L, LOW);

  for (int pwmValue = startPWM; pwmValue <= endPWM; pwmValue += pwmIncrement) { // Sweep PWM
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

  // --- Test Right Motors (Forward) ---
  Serial.println("\n--- Testing Right Motors: FORWARD (PWM from " + String(startPWM) + " to " + String(endPWM) + ") ---");
  digitalWrite(IN3R, HIGH); // Set direction forward
  digitalWrite(IN4R, LOW);

  for (int pwmValue = startPWM; pwmValue <= endPWM; pwmValue += pwmIncrement) { // Sweep PWM
    analogWrite(ENAR, pwmValue); // Apply PWM
    Serial.print("Right Forward PWM: ");
    Serial.println(pwmValue);
    delay(sweepDelay);
  }
  delay(stopDelay); // Wait at full speed

  // Stop Right Motors
  Serial.println("Stopping Right Motors.");
  analogWrite(ENAR, 0);
  delay(stopDelay * 2); // Longer stop before repeating the whole sequence

  // To simplify the test and focus on startup, I've removed backward sweep for brevity.
  // You can re-add if needed, following the same pattern for IN1L/IN2L and IN3R/IN4R logic.
}