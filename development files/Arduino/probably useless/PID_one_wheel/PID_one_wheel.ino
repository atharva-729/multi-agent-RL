// --- Motor Driver Pins (L298N) ---
// Left Motor (Motor A) - Using pins from your previous working code
const int ENAL = 9;  // PWM pin for Left Motor speed (Connect to L298N ENA)
const int IN1L = 8;  // Left Motor Direction Pin 1 (Connect to L298N IN1)
const int IN2L = 7;  // Left Motor Direction Pin 2 (Connect to L298N IN2)

// Right Motor (Motor B) - Using pins from your previous working code.
const int ENAR = 10; // PWM pin for Right Motor speed (Connect to L298N ENB)
const int IN3R = 5;  // Right Motor Direction Pin 1 (Connect to L298N IN3)
const int IN4R = 4;  // Right Motor Direction Pin 2 (Connect to L298N IN4)

// --- Encoder Pins and Variables ---
// IMPORTANT: Using Arduino Mega interrupt pins from your previous working code (2 and 3)
const int LEFT_ENCODER_PIN = 2;  // Connect Left Encoder to Digital Pin 2 (Interrupt 0)
const int RIGHT_ENCODER_PIN = 3; // Connect Right Encoder to Digital Pin 3 (Interrupt 1)

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16; // Based on your 8 black stripes and effective 16 pulses per revolution
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI;

// --- Debouncing Variables for Encoders ---
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;
const unsigned long DEBOUNCE_DELAY_MS = 20; // Adjust if needed for stable readings

// Timing for RPM calculation and PID loop
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 1000; // PID loop updates every 1 second

// --- Timing Variables for Serial Printing ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 1000; // Print every 1 second to match PID loop

float leftRPM = 0.0;
float rightRPM = 0.0;

// --- PID Control Variables for Right Motor ---
float Kp = 3.0;  // Proportional gain
float Ki = 0.00; // Integral gain
float Kd = 0.00; // Derivative gain

float targetRPM_Right = 75.0; // <--- SET YOUR DESIRED TARGET RPM FOR THE RIGHT MOTOR HERE!
float currentRPM_Right = 0.0; // This variable will now hold the RPM of the RIGHT motor

float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;
float outputPWM_Right = 0.0;

// --- PWM Output Limits ---
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// --- NEW: Minimum Effective PWM (Motor Deadband Compensation) ---
// This is the minimum PWM value required for your motor to actually start spinning.
// You need to find this value by trial and error.
// Start with 0, then increase until your motor reliably starts with a direct analogWrite().
const int MIN_EFFECTIVE_PWM = 40; // <--- ADJUST THIS VALUE! (e.g., try 30, 40, 50, 60)
// --- END NEW ---


void setup() {
  Serial.begin(9600);
  Serial.println("Robot Motor PID Control Test");
  Serial.print("Target Right RPM: ");
  Serial.println(targetRPM_Right);
  Serial.println("----------------------------------------");
  Serial.println("WARNING: PID loop updates every 1 second. Control may be sluggish.");
  Serial.println("Controlling the RIGHT Motor.");
  Serial.print("MIN_EFFECTIVE_PWM set to: ");
  Serial.println(MIN_EFFECTIVE_PWM);

  // Motor Driver Pin Setup
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder Pin Setup
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize motors to stop
  stopMotors();
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  // --- PID Loop Execution (now every 1 second) ---
  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // 1. Read Encoders and Calculate Current RPM for the RIGHT motor
    noInterrupts();
    long currentRightTicks_copy = rightEncoderTicks; // Reading RIGHT encoder ticks
    rightEncoderTicks = 0; // Reset right encoder ticks
    interrupts();

    currentRPM_Right = (float)currentRightTicks_copy / PPR / (deltaTime_ms / 60000.0);

    // 2. PID Calculation for Right Motor
    error_Right = targetRPM_Right - currentRPM_Right;

    float P_term = Kp * error_Right;

    integral_Right += error_Right * (deltaTime_ms / 1000.0);
    // Anti-windup clamping for integral term
    if ( (outputPWM_Right >= MAX_PWM && error_Right > 0) || (outputPWM_Right <= MIN_PWM && error_Right < 0) ) {
        // Do not accumulate integral if output is saturated and error tries to push it further
    } else {
        if (integral_Right > 500) integral_Right = 500;
        if (integral_Right < -500) integral_Right = -500;
    }
    float I_term = Ki * integral_Right;

    derivative_Right = (error_Right - prevError_Right) / (deltaTime_ms / 1000.0);
    float D_term = Kd * derivative_Right; // Corrected: was derivative_Left

    outputPWM_Right = P_term + I_term + D_term;

    // --- Apply Minimum Effective PWM (Deadband Compensation) ---
    // If the calculated PWM is positive but too low to start the motor,
    // boost it to the minimum effective PWM.
    if (outputPWM_Right > 0 && outputPWM_Right < MIN_EFFECTIVE_PWM) {
        outputPWM_Right = MIN_EFFECTIVE_PWM;
    }
    // --- END Deadband Compensation ---

    // Clamp PWM output to valid range (0-255)
    if (outputPWM_Right > MAX_PWM) outputPWM_Right = MAX_PWM;
    if (outputPWM_Right < MIN_PWM) outputPWM_Right = MIN_PWM;

    prevError_Right = error_Right;

    // 3. Apply PWM to the RIGHT Motor
    digitalWrite(IN3R, HIGH); // Set RIGHT Motor direction (Forward)
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right); // Apply PWM to RIGHT Motor

    // 4. Ensure LEFT Motor is OFF
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, 0); // Ensure left motor is off

    // --- Serial Monitor Printing (now coincides with PID loop) ---
    Serial.print("Target RPM: ");
    Serial.print(targetRPM_Right);
    Serial.print(" | Current RPM (Right): ");
    Serial.print(currentRPM_Right);
    Serial.print(" | Error: ");
    Serial.print(error_Right);
    Serial.print(" | PWM (Right): ");
    Serial.println((int)outputPWM_Right);

    lastPrintTime = currentTime;
  }
}

// --- Motor Control Helper Functions ---
void stopMotors() {
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0);

  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
}

// --- Encoder Interrupt Service Routines (ISRs) ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++;
    lastPulseTimeLeft = currentTime;
  }
}

void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}