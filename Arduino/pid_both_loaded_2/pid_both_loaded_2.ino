// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 10; // PWM pin for Left Motor speed
const int IN1L = 5;  // Left Motor Direction Pin 1
const int IN2L = 4;  // Left Motor Direction Pin 2

// Right Motor pins
const int ENAR = 9;  // PWM pin for Right Motor speed
const int IN3R = 8;  // Right Motor Direction Pin 1
const int IN4R = 7;  // Right Motor Direction Pin 2

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16;
const unsigned long DEBOUNCE_DELAY_MS = 20;
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// Timing for RPM calculation and PID loop
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 200ms

// --- Timing Variables for Serial Printing ---
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 200; // Print every 200ms to match PID loop

// --- PID Control Variables for LEFT Motor ---
// Use your best tuned values for the Left motor here as starting point
float Kp_L = 0.4;  // Proportional gain for Left Motor
float Ki_L = 0.0;  // Integral gain for Left Motor
float Kd_L = 0.0;  // Derivative gain for Left Motor

float targetRPM_Left = 75.0; // Desired target RPM for Left Motor
float currentRPM_Left = 0.0;
float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;
float outputPWM_Left = 0.0;

// --- PID Control Variables for RIGHT Motor ---
// Start with conservative values for the Right motor based on its earlier sensitivity
float Kp_R = 0.2;  // Proportional gain for Right Motor (start lower than Left)
float Ki_R = 0.0;  // Integral gain for Right Motor
float Kd_R = 0.0;  // Derivative gain for Right Motor

float targetRPM_Right = 75.0; // Desired target RPM for Right Motor
float currentRPM_Right = 0.0;
float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;
float outputPWM_Right = 0.0;


// --- Motor Characterization (Regression Lines - LOADED) ---
// From your latest analysis:
// Left Motor: RPM = 0.3398 * PWM + 14.5270
const float REGRESSION_SLOPE_L = 0.3398;
const float REGRESSION_INTERCEPT_L = 14.5270;

// Right Motor: RPM = 0.3750 * PWM + 16.3585
const float REGRESSION_SLOPE_R = 0.3750;
const float REGRESSION_INTERCEPT_R = 16.3585;

// --- PWM Output Limits ---
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// --- Minimum Effective PWM (Motor Deadband Compensation - LOADED) ---
// From your latest analysis:
const int MIN_EFFECTIVE_PWM_L = 70; // For Left Motor
const int MIN_EFFECTIVE_PWM_R = 70; // For Right Motor


void setup() {
  Serial.begin(9600);
  Serial.println("Robot Dual Motor PID Control Test (Loaded)");
  Serial.print("Target Left RPM: "); Serial.print(targetRPM_Left);
  Serial.print(" | Target Right RPM: "); Serial.println(targetRPM_Right);
  Serial.println("----------------------------------------");
  Serial.println("PID loop updates every 200ms.");
  Serial.println("Controlling BOTH Motors with Feedforward.");
  Serial.print("MIN_EFFECTIVE_PWM_L: "); Serial.print(MIN_EFFECTIVE_PWM_L);
  Serial.print(" | MIN_EFFECTIVE_PWM_R: "); Serial.println(MIN_EFFECTIVE_PWM_R);
  Serial.print("Kp_L: "); Serial.print(Kp_L); Serial.print(", Ki_L: "); Serial.print(Ki_L); Serial.print(", Kd_L: "); Serial.println(Kd_L);
  Serial.print("Kp_R: "); Serial.print(Kp_R); Serial.print(", Ki_R: "); Serial.print(Ki_R); Serial.print(", Kd_R: "); Serial.println(Kd_R);


  // Motor Driver Pin Setup for LEFT Motor
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Motor Driver Pin Setup for RIGHT Motor
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder Pin Setup for BOTH Encoders
  pinMode(LEFT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize both motors to stop
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
  delay(1000);
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  // --- PID Loop Execution for BOTH motors ---
  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // --- LEFT Motor PID ---
    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    leftEncoderTicks = 0;
    interrupts();

    currentRPM_Left = (float)currentLeftTicks_copy / PPR / (deltaTime_ms / 60000.0);
    error_Left = targetRPM_Left - currentRPM_Left;

    float base_pwm_L = (targetRPM_Left - REGRESSION_INTERCEPT_L) / REGRESSION_SLOPE_L;
    // Cap base_pwm_L to avoid extreme initial values if target is very low
    if (base_pwm_L < 0) base_pwm_L = 0; // If target is below motor's minimum, base is 0
    if (base_pwm_L > MAX_PWM) base_pwm_L = MAX_PWM; // Don't exceed max PWM

    // PID terms for correction
    float P_term_L = Kp_L * error_Left;

    if (Ki_L > 0.0) { // Only accumulate integral if Ki is active
      integral_Left += error_Left * (deltaTime_ms / 1000.0);
      if ( (outputPWM_Left >= MAX_PWM && error_Left > 0) || (outputPWM_Left <= MIN_PWM && error_Left < 0) ) { /* anti-windup */ }
      else { if (integral_Left > 500) integral_Left = 500; if (integral_Left < -500) integral_Left = -500; }
    } else { integral_Left = 0; }
    float I_term_L = Ki_L * integral_Left;

    derivative_Left = (error_Left - prevError_Left) / (deltaTime_ms / 1000.0);
    float D_term_L = Kd_L * derivative_Left;

    if (outputPWM_Left == 0.0) outputPWM_Left = base_pwm_L;

    outputPWM_Left = outputPWM_Left + P_term_L + I_term_L + D_term_L;

    // Apply Minimum Effective PWM (Deadband Compensation)
    if (targetRPM_Left > 0 && outputPWM_Left > 0 && outputPWM_Left < MIN_EFFECTIVE_PWM_L) { outputPWM_Left = MIN_EFFECTIVE_PWM_L; }
    if (targetRPM_Left == 0) { outputPWM_Left = 0; } // Ensure off if target is 0

    // Clamp final PWM output to valid range (0-255)
    if (outputPWM_Left > MAX_PWM) outputPWM_Left = MAX_PWM;
    if (outputPWM_Left < MIN_PWM) outputPWM_Left = MIN_PWM;
    prevError_Left = error_Left;


    // --- RIGHT Motor PID ---
    noInterrupts();
    long currentRightTicks_copy = rightEncoderTicks;
    rightEncoderTicks = 0;
    interrupts();

    currentRPM_Right = (float)currentRightTicks_copy / PPR / (deltaTime_ms / 60000.0);
    error_Right = targetRPM_Right - currentRPM_Right;

    float base_pwm_R = (targetRPM_Right - REGRESSION_INTERCEPT_R) / REGRESSION_SLOPE_R;
    // Cap base_pwm_R to avoid extreme initial values if target is very low
    if (base_pwm_R < 0) base_pwm_R = 0; // If target is below motor's minimum, base is 0
    if (base_pwm_R > MAX_PWM) base_pwm_R = MAX_PWM; // Don't exceed max PWM

    // PID terms for correction
    float P_term_R = Kp_R * error_Right;

    if (Ki_R > 0.0) { // Only accumulate integral if Ki is active
      integral_Right += error_Right * (deltaTime_ms / 1000.0);
      if ( (outputPWM_Right >= MAX_PWM && error_Right > 0) || (outputPWM_Right <= MIN_PWM && error_Right < 0) ) { /* anti-windup */ }
      else { if (integral_Right > 500) integral_Right = 500; if (integral_Right < -500) integral_Right = -500; }
    } else { integral_Right = 0; }
    float I_term_R = Ki_R * integral_Right;

    derivative_Right = (error_Right - prevError_Right) / (deltaTime_ms / 1000.0);
    float D_term_R = Kd_R * derivative_Right;

    if (outputPWM_Right == 0.0) outputPWM_Right = base_pwm_R;

    outputPWM_Right = outputPWM_Right + P_term_R + I_term_R + D_term_R;

    // Apply Minimum Effective PWM (Deadband Compensation)
    if (targetRPM_Right > 0 && outputPWM_Right > 0 && outputPWM_Right < MIN_EFFECTIVE_PWM_R) { outputPWM_Right = MIN_EFFECTIVE_PWM_R; }
    if (targetRPM_Right == 0) { outputPWM_Right = 0; } // Ensure off if target is 0

    // Clamp final PWM output to valid range (0-255)
    if (outputPWM_Right > MAX_PWM) outputPWM_Right = MAX_PWM;
    if (outputPWM_Right < MIN_PWM) outputPWM_Right = MIN_PWM;
    prevError_Right = error_Right;


    // --- Apply PWM to BOTH Motors ---
    digitalWrite(IN1L, HIGH); // Left Motor Forward
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left);

    digitalWrite(IN3R, HIGH); // Right Motor Forward
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right);

    // --- Serial Monitor Printing ---
    Serial.print("TRPM: "); Serial.print(targetRPM_Left);
    Serial.print(" | CRPM_L: "); Serial.print(currentRPM_Left);
    Serial.print(" | Err_L: "); Serial.print(error_Left);
    Serial.print(" | PWM_L: "); Serial.print((int)outputPWM_Left);
    Serial.print(" | CRPM_R: "); Serial.print(currentRPM_Right);
    Serial.print(" | Err_R: "); Serial.print(error_Right);
    Serial.print(" | PWM_R: "); Serial.println((int)outputPWM_Right);

    lastPrintTime = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for LEFT Encoder ---
void leftEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    leftEncoderTicks++;
    lastPulseTimeLeft = currentTime;
  }
}

// --- Encoder Interrupt Service Routine (ISR) for RIGHT Encoder ---
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}