// --- Motor Driver Pins (L298N) ---
// Left Motor pins (will be kept off in this sketch)
const int ENAL = 9; // PWM pin for Left Motor speed (Connect to L298N ENB)
const int IN1L = 8;  // Left Motor Direction Pin 1 (Connect to L298N IN3)
const int IN2L = 7;  // Left Motor Direction Pin 2 (Connect to L298N IN4)

// Right Motor pins (controlled in this sketch)
const int ENAR = 10;  // PWM pin for Right Motor speed
const int IN3R = 5;  // Right Motor Direction Pin 1
const int IN4R = 4;  // Right Motor Direction Pin 2

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;  // Ticks for the LEFT encoder (not actively used for control in this sketch)
volatile long rightEncoderTicks = 0; // Ticks for the RIGHT encoder

// Encoder properties
const int PPR = 16;
const unsigned long DEBOUNCE_DELAY_MS = 20;
volatile unsigned long lastPulseTimeLeft = 0; // For Left Encoder (not actively used for control)
volatile unsigned long lastPulseTimeRight = 0; // For Right Encoder

// Timing for RPM calculation and PID loop
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 200ms

// --- Timing Variables for Serial Printing ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 200; // Print every 200ms to match PID loop

// --- PID Control Variables for Right Motor ---
// These are for the *correction* part of the PID
// Start with initial values from Left Motor tuning (or adjust to your baseline for Right)
float Kp = 1.0;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.0;  // Derivative gain

float targetRPM_Right = 75.0; // Your desired target RPM for the Right Motor
float currentRPM_Right = 0.0;

float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;

// This will be the final PWM value applied to the motor
float outputPWM_Right = 0.0;

// --- Motor Characterization (Regression Line for Right Motor) ---
// RPM = 0.3320 * PWM + 33.1979
const float REGRESSION_SLOPE = 0.3320; // Updated for Right Motor
const float REGRESSION_INTERCEPT = 33.1979; // Updated for Right Motor

// --- PWM Output Limits ---
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
// This is the minimum PWM value required for your motor to actually start spinning.
// You need to determine this value experimentally for the RIGHT motor.
const int MIN_EFFECTIVE_PWM = 60; // <--- ADJUST THIS VALUE! (might be different for Right Motor)

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Motor PID Control Test (Right Motor with Feedforward)");
  Serial.print("Target Right RPM: ");
  Serial.println(targetRPM_Right);
  Serial.println("----------------------------------------");
  Serial.println("PID loop updates every 200ms.");
  Serial.println("Controlling the RIGHT Motor with Feedforward.");
  Serial.print("MIN_EFFECTIVE_PWM set to: ");
  Serial.println(MIN_EFFECTIVE_PWM);

  // Motor Driver Pin Setup for RIGHT Motor
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Ensure LEFT Motor pins are set to output and off
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0); // Stop Left Motor

  // Encoder Pin Setup for RIGHT Encoder
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize Right motor to stop
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0); // Stop Right Motor
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  // --- PID Loop Execution ---
  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // 1. Read Right Encoder and Calculate Current RPM
    noInterrupts();
    long currentRightTicks_copy = rightEncoderTicks;
    rightEncoderTicks = 0; // Reset right encoder ticks
    interrupts();

    currentRPM_Right = (float)currentRightTicks_copy / PPR / (deltaTime_ms / 60000.0);

    // 2. PID Calculation for Right Motor with Feedforward
    error_Right = targetRPM_Right - currentRPM_Right;

    // Calculate Feedforward PWM
    // This is the base PWM predicted by your regression line for the target RPM
    float base_pwm_R = (targetRPM_Right - REGRESSION_INTERCEPT) / REGRESSION_SLOPE;
    // Only apply base_pwm as initial guess if outputPWM_Right is currently 0
    if (outputPWM_Right == 0) outputPWM_Right = base_pwm_R;

    // PID terms for correction
    float P_term_R = Kp * error_Right;

    integral_Right += error_Right * (deltaTime_ms / 1000.0);
    // Anti-windup clamping for integral term
    if ( (outputPWM_Right >= MAX_PWM && error_Right > 0) || (outputPWM_Right <= MIN_PWM && error_Right < 0) ) {
        // Do not accumulate integral if output is saturated and error tries to push it further
    } else {
        if (integral_Right > 500) integral_Right = 500; // Arbitrary limits for integral windup prevention
        if (integral_Right < -500) integral_Right = -500;
    }
    float I_term_R = Ki * integral_Right;

    derivative_Right = (error_Right - prevError_Right) / (deltaTime_ms / 1000.0);
    float D_term_R = Kd * derivative_Right;

    // Combine Feedforward and PID Correction
    outputPWM_Right = outputPWM_Right + P_term_R + I_term_R + D_term_R;

    // --- Apply Minimum Effective PWM (Deadband Compensation) ---
    if (outputPWM_Right > 0 && outputPWM_Right < MIN_EFFECTIVE_PWM) {
        outputPWM_Right = MIN_EFFECTIVE_PWM;
    }
    // --- END Deadband Compensation ---

    // Clamp final PWM output to valid range (0-255)
    if (outputPWM_Right > MAX_PWM) outputPWM_Right = MAX_PWM;
    if (outputPWM_Right < MIN_PWM) outputPWM_Right = MIN_PWM;

    prevError_Right = error_Right;

    // 3. Apply PWM to the RIGHT Motor
    digitalWrite(IN3R, HIGH); // Set RIGHT Motor direction (Forward)
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right); // Apply PWM to RIGHT Motor

    // 4. Ensure Left Motor is OFF
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, 0);

    // --- Serial Monitor Printing ---
    Serial.print("Target RPM: ");
    Serial.print(targetRPM_Right);
    Serial.print(" | Current RPM (Right): ");
    Serial.print(currentRPM_Right);
    Serial.print(" | Error: ");
    Serial.print(error_Right);
    Serial.print(" | Base PWM: ");
    Serial.print(base_pwm_R);
    Serial.print(" | Final PWM (Right): ");
    Serial.println((int)outputPWM_Right);

    lastPrintTime = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for LEFT Encoder (not used for control in this sketch) ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++; // Increment ticks for the LEFT encoder
    lastPulseTimeLeft = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for RIGHT Encoder ---
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++; // Increment ticks for the RIGHT encoder
    lastPulseTimeRight = currentTime;
  }
}