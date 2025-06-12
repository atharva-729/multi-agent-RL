// --- Motor Driver Pins (L298N) ---
// Left Motor pins (controlled in this sketch)
const int ENAL = 10; // PWM pin for Left Motor speed (Connect to L298N ENB)
const int IN1L = 5;  // Left Motor Direction Pin 1 (Connect to L298N IN3)
const int IN2L = 4;  // Left Motor Direction Pin 2 (Connect to L298N IN4)

// Right Motor pins (will be kept off in this sketch)
const int ENAR = 9;  // PWM pin for Right Motor speed
const int IN3R = 8;  // Right Motor Direction Pin 1
const int IN4R = 7;  // Right Motor Direction Pin 2

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0; // Ticks for the LEFT encoder
volatile long rightEncoderTicks = 0; // Ticks for the RIGHT encoder (not actively used for control in this sketch)

// Encoder properties
const int PPR = 16;
const unsigned long DEBOUNCE_DELAY_MS = 20;
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// Timing for RPM calculation and PID loop
unsigned long lastLoopTime = 0;
// You can adjust this back to a smaller value (e.g., 50ms-100ms) once you confirm
// this controller is working well.
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 1 second

// --- Timing Variables for Serial Printing ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 200; // Print every 1 second to match PID loop

// --- PID Control Variables for Left Motor ---
// These are for the *correction* part of the PID
float Kp = 0.4;   // Proportional gain: Start with a smaller Kp when using feedforward
float Ki = 0.0;  // Integral gain: Introduce a small Ki to eliminate steady-state error
float Kd = 0.0;  // Derivative gain: Keep 0 for now, introduce later for damping

float targetRPM_Left = 75.0; // Your desired target RPM for the Left Motor
float currentRPM_Left = 0.0;

float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;

// This will be the final PWM value applied to the motor
float outputPWM_Left = 0.0;

// --- Motor Characterization (Regression Line for Left Motor) ---
// RPM = 0.3830 * PWM + 23.3868
// PWM = (RPM - 23.3868) / 0.3830
const float REGRESSION_SLOPE = 0.3830;
const float REGRESSION_INTERCEPT = 23.3868;

// --- PWM Output Limits ---
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
// This is the minimum PWM value required for your motor to actually start spinning.
// You need to determine this value experimentally for the LEFT motor.
const int MIN_EFFECTIVE_PWM = 60; // <--- ADJUST THIS VALUE! (e.g., try 40, 50, 60, 70)

void setup() {
  Serial.begin(9600);
  Serial.println("Robot Motor PID Control Test (Left Motor with Feedforward)");
  Serial.print("Target Left RPM: ");
  Serial.println(targetRPM_Left);
  Serial.println("----------------------------------------");
  Serial.println("WARNING: PID loop updates every 1 second. Control may be sluggish.");
  Serial.println("Controlling the LEFT Motor with Feedforward.");
  Serial.print("MIN_EFFECTIVE_PWM set to: ");
  Serial.println(MIN_EFFECTIVE_PWM);

  // Motor Driver Pin Setup for LEFT Motor (using corrected pins)
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Ensure RIGHT Motor pins are set to output and off (using corrected pins for Right)
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);

  // Encoder Pin Setup for LEFT Encoder (using corrected pin)
  pinMode(LEFT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);

  // Initialize motors to stop
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0); // Stop Left Motor
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  // --- PID Loop Execution (currently every 1 second) ---
  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // 1. Read Left Encoder and Calculate Current RPM
    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    leftEncoderTicks = 0; // Reset left encoder ticks
    interrupts();

    currentRPM_Left = (float)currentLeftTicks_copy / PPR / (deltaTime_ms / 60000.0);

    // 2. PID Calculation for Left Motor with Feedforward
    error_Left = targetRPM_Left - currentRPM_Left;

    // Calculate Feedforward PWM
    // This is the base PWM predicted by your regression line for the target RPM
    float base_pwm_L = (targetRPM_Left - REGRESSION_INTERCEPT) / REGRESSION_SLOPE;
    if (outputPWM_Left == 0) outputPWM_Left = base_pwm_L;  

    // PID terms for correction
    float P_term_L = Kp * error_Left;

    integral_Left += error_Left * (deltaTime_ms / 1000.0);
    // Anti-windup clamping for integral term
    if ( (outputPWM_Left >= MAX_PWM && error_Left > 0) || (outputPWM_Left <= MIN_PWM && error_Left < 0) ) {
        // Do not accumulate integral if output is saturated and error tries to push it further
    } else {
        if (integral_Left > 500) integral_Left = 500; // Arbitrary limits for integral windup prevention
        if (integral_Left < -500) integral_Left = -500;
    }
    float I_term_L = Ki * integral_Left;

    derivative_Left = (error_Left - prevError_Left) / (deltaTime_ms / 1000.0);
    float D_term_L = Kd * derivative_Left;

    // Combine Feedforward and PID Correction
    outputPWM_Left = outputPWM_Left + P_term_L + I_term_L + D_term_L;

    // --- Apply Minimum Effective PWM (Deadband Compensation) ---
    if (outputPWM_Left > 0 && outputPWM_Left < MIN_EFFECTIVE_PWM) {
        outputPWM_Left = MIN_EFFECTIVE_PWM;
    }
    // --- END Deadband Compensation ---

    // Clamp final PWM output to valid range (0-255)
    if (outputPWM_Left > MAX_PWM) outputPWM_Left = MAX_PWM;
    if (outputPWM_Left < MIN_PWM) outputPWM_Left = MIN_PWM;

    prevError_Left = error_Left;

    // 3. Apply PWM to the LEFT Motor
    digitalWrite(IN1L, HIGH); // Set LEFT Motor direction (Forward)
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left); // Apply PWM to LEFT Motor

    // 4. Ensure Right Motor is OFF (using corrected pins for Right)
    digitalWrite(IN3R, LOW);
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, 0);

    // --- Serial Monitor Printing ---
    Serial.print("Target RPM: ");
    Serial.print(targetRPM_Left);
    Serial.print(" | Current RPM (Left): ");
    Serial.print(currentRPM_Left);
    Serial.print(" | Error: ");
    Serial.print(error_Left);
    Serial.print(" | Base PWM: ");
    Serial.print(base_pwm_L);
    Serial.print(" | Final PWM (Left): ");
    Serial.println((int)outputPWM_Left);

    lastPrintTime = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for LEFT Encoder ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++; // Increment ticks for the LEFT encoder
    lastPulseTimeLeft = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for RIGHT Encoder ---
// This ISR is defined but not attached if you're only controlling the left motor.
// It's here for completeness for a dual-encoder setup.
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++; // Increment ticks for the RIGHT encoder
    lastPulseTimeRight = currentTime;
  }
}