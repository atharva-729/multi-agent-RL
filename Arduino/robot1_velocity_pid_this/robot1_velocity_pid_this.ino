#include <Arduino.h> // Standard Arduino library

// --- Bluetooth Module Pins (HC-05 using Hardware Serial1 on Mega) ---
// On Arduino Mega:
// RX1 (Pin 19) will receive data from HC-05 TX
// TX1 (Pin 18) will send data to HC-05 RX (via voltage divider)
// Ensure your HC-05's baud rate is set to 9600 (default) or match it here.

// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 9;  // PWM pin for Left Motor speed (Connect to L298N ENB)
const int IN1L = 8;  // Left Motor Direction Pin 1 (Connect to L298N IN3)
const int IN2L = 7;  // Left Motor Direction Pin 2 (Connect to L298N IN4)

// Right Motor pins
const int ENAR = 10; // PWM pin for Right Motor speed (Connect to L298N ENA)
const int IN3R = 5;  // Right Motor Direction Pin 1 (Connect to L298N IN1)
const int IN4R = 4;  // Right Motor Direction Pin 2 (Connect to L298N IN2)

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16; // Pulses Per Revolution of your encoder
const unsigned long DEBOUNCE_DELAY_MS = 20; // Debounce for encoder interrupts
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// --- Timing for RPM calculation and PID loop ---
unsigned long lastLoopTime = 0;
const unsigned long LOOP_INTERVAL_MS = 200; // PID loop updates every 200ms

// --- PID Control Variables for LEFT Motor ---
float Kp_L = 0.6;  // Proportional gain for Left Motor (adjust as needed)
float Ki_L = 0.0;  // Integral gain for Left Motor (start with 0.0, tune later)
float Kd_L = 0.2;  // Derivative gain for Left Motor (start with 0.0, tune later)

float targetRPM_Left = 18.75; // Desired target RPM for Left Motor
float currentRPM_Left = 0.0;
float error_Left = 0.0;
float prevError_Left = 0.0;
float integral_Left = 0.0;
float derivative_Left = 0.0;
float outputPWM_Left = 0.0; // This variable will be cumulatively updated

// --- PID Control Variables for RIGHT Motor ---
float Kp_R = 0.6;  // Proportional gain for Right Motor
float Ki_R = 0.0;  // Integral gain for Right Motor
float Kd_R = 0.3;  // Derivative gain for Right Motor

float targetRPM_Right = 37.5; // Desired target RPM for Right Motor
float currentRPM_Right = 0.0;
float error_Right = 0.0;
float prevError_Right = 0.0;
float integral_Right = 0.0;
float derivative_Right = 0.0;
float outputPWM_Right = 0.0; // This variable will be cumulatively updated

// --- Motor Characterization (Regression Lines) ---
const float REGRESSION_SLOPE_L = 0.4523;
const float REGRESSION_INTERCEPT_L = -3.0138;

const float REGRESSION_SLOPE_R = 0.3636;
const float REGRESSION_INTERCEPT_R = 12.2727;

// --- PWM Output Limits ---
const int MIN_PWM_OVERALL = 0; // Overall minimum PWM (0 for off)
const int MAX_PWM_OVERALL = 255; // Overall maximum PWM

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
const int MIN_EFFECTIVE_PWM_L = 85; // For Left Motor (adjust if needed for 9V)
const int MIN_EFFECTIVE_PWM_R = 85; // For Right Motor (adjust if needed for 9V)


void setup() {
  // Initialize USB Serial for debugging (to your laptop's Arduino IDE Serial Monitor)
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open (useful for some boards)
  Serial.println("Robot Dual Motor PID Control Test");
  Serial.print("Target Left RPM: "); Serial.print(targetRPM_Left);
  Serial.print(" | Target Right RPM: "); Serial.println(targetRPM_Right);
  Serial.println("----------------------------------------");
  Serial.print("PID loop updates every "); Serial.print(LOOP_INTERVAL_MS); Serial.println("ms.");
  Serial.println("Controlling BOTH Motors with Feedforward.");
  Serial.print("MIN_EFFECTIVE_PWM_L: "); Serial.print(MIN_EFFECTIVE_PWM_L);
  Serial.print(" | MIN_EFFECTIVE_PWM_R: "); Serial.print(MIN_EFFECTIVE_PWM_R);
  Serial.print(" | Kp_L: "); Serial.print(Kp_L); Serial.print(", Ki_L: "); Serial.print(Ki_L); Serial.print(", Kd_L: "); Serial.println(Kd_L);
  Serial.print(" | Kp_R: "); Serial.print(Kp_R); Serial.print(", Ki_R: "); Serial.print(Ki_R); Serial.print(", Kd_R: "); Serial.println(Kd_R);


  // Initialize Hardware Serial1 for HC-05 Bluetooth communication
  Serial1.begin(9600); // HC-05 default baud rate is 9600
  Serial1.println("Arduino: HC-05 Ready for Data Streaming!");

  // Motor Driver Pin Setup for LEFT Motor
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Motor Driver Pin Setup for RIGHT Motor
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder Pin Setup for BOTH Encoders
  // Digital Pin 3 (Interrupt 1) and Digital Pin 2 (Interrupt 0) are external interrupts on Arduino Mega
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP); // Use INPUT_PULLUP if encoder outputs open-collector/open-drain
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE); // Trigger on any state change
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Initialize both motors to stop
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
  delay(15000); // Short delay to allow everything to initialize
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime_ms = (float)(currentTime - lastLoopTime);

  // --- Process incoming Bluetooth commands (ONLY "STOP") ---
  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n'); // Read incoming command until newline
    command.trim(); // Remove any leading/trailing whitespace

    if (command == "STOP") {
      targetRPM_Left = 0.0;
      targetRPM_Right = 0.0;
      // Immediately set PWM to 0 to stop
      digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
      digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
      outputPWM_Left = 0.0; // Reset cumulative PWM
      outputPWM_Right = 0.0; // Reset cumulative PWM
      integral_Left = 0.0;   // Reset integral terms
      integral_Right = 0.0;
      Serial.println("Motors STOPPED by BT command!"); // Debug via USB
    }
  }


  // --- PID Loop Execution for BOTH motors ---
  if (deltaTime_ms >= LOOP_INTERVAL_MS) {
    lastLoopTime = currentTime;

    // --- LEFT Motor PID Calculation ---
    // Disable interrupts briefly to read volatile variables safely
    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    leftEncoderTicks = 0; // Reset ticks for the next interval
    interrupts();

    currentRPM_Left = (float)currentLeftTicks_copy / PPR / (deltaTime_ms / 60000.0); // Calculate RPM
    error_Left = targetRPM_Left - currentRPM_Left;

    // Calculate Feedforward PWM
    float base_pwm_L = (targetRPM_Left - REGRESSION_INTERCEPT_L) / REGRESSION_SLOPE_L;
    // Clamp base_pwm_L to avoid extreme values
    if (base_pwm_L < MIN_PWM_OVERALL) base_pwm_L = MIN_PWM_OVERALL;
    if (base_pwm_L > MAX_PWM_OVERALL) base_pwm_L = MAX_PWM_OVERALL;

    // PID terms for correction
    float P_term_L = Kp_L * error_Left;

    // Integral term with anti-windup
    if (Ki_L > 0.0) { 
      integral_Left += error_Left * (deltaTime_ms / 1000.0);
      // Basic anti-windup: only accumulate integral if not saturated
      if (!((outputPWM_Left >= MAX_PWM_OVERALL && error_Left > 0) || (outputPWM_Left <= MIN_PWM_OVERALL && error_Left < 0))) { 
        // Clamp integral to prevent excessive accumulation
        if (integral_Left > 500) integral_Left = 500; 
        if (integral_Left < -500) integral_Left = -500; 
      }
    } else { 
      integral_Left = 0; // If Ki is 0, reset integral
    }
    float I_term_L = Ki_L * integral_Left;

    // Derivative term
    derivative_Left = (error_Left - prevError_Left) / (deltaTime_ms / 1000.0);
    float D_term_L = Kd_L * derivative_Left;

    // --- YOUR ITERATIVE/CUMULATIVE PID UPDATE LOGIC ---
    // Initialize outputPWM_Left with base_pwm_L only if it's currently 0 (first run or motor off)
    // For fixed targets, this means it initializes once. For external target changes, you'd add a flag.
    if (outputPWM_Left == 0.0 && targetRPM_Left > 0) { // Only initialize with base_pwm if we're trying to move
        outputPWM_Left = base_pwm_L;
    }
    
    // Incrementally adjust outputPWM_Left based on PID terms
    outputPWM_Left = outputPWM_Left + P_term_L + I_term_L + D_term_L;


    // Apply Minimum Effective PWM (Deadband Compensation)
    if (targetRPM_Left > 0 && outputPWM_Left > 0 && outputPWM_Left < MIN_EFFECTIVE_PWM_L) { outputPWM_Left = MIN_EFFECTIVE_PWM_L; }
    else if (targetRPM_Left == 0) { outputPWM_Left = 0; } // Ensure off if target is 0

    // Clamp final PWM output to valid range (0-255)
    if (outputPWM_Left > MAX_PWM_OVERALL) outputPWM_Left = MAX_PWM_OVERALL; 
    if (outputPWM_Left < MIN_PWM_OVERALL) outputPWM_Left = MIN_PWM_OVERALL;
    prevError_Left = error_Left;


    // --- RIGHT Motor PID Calculation ---
    noInterrupts();
    long currentRightTicks_copy = rightEncoderTicks;
    rightEncoderTicks = 0;
    interrupts();

    currentRPM_Right = (float)currentRightTicks_copy / PPR / (deltaTime_ms / 60000.0);
    error_Right = targetRPM_Right - currentRPM_Right;

    float base_pwm_R = (targetRPM_Right - REGRESSION_INTERCEPT_R) / REGRESSION_SLOPE_R;
    if (base_pwm_R < MIN_PWM_OVERALL) base_pwm_R = MIN_PWM_OVERALL;
    if (base_pwm_R > MAX_PWM_OVERALL) base_pwm_R = MAX_PWM_OVERALL;

    float P_term_R = Kp_R * error_Right;

    if (Ki_R > 0.0) {
      integral_Right += error_Right * (deltaTime_ms / 1000.0);
      if (!((outputPWM_Right >= MAX_PWM_OVERALL && error_Right > 0) || (outputPWM_Right <= MIN_PWM_OVERALL && error_Right < 0))) { 
        if (integral_Right > 500) integral_Right = 500; 
        if (integral_Right < -500) integral_Right = -500; 
      }
    } else { 
      integral_Right = 0; 
    }
    float I_term_R = Ki_R * integral_Right;

    derivative_Right = (error_Right - prevError_Right) / (deltaTime_ms / 1000.0);
    float D_term_R = Kd_R * derivative_Right;

    // --- YOUR ITERATIVE/CUMULATIVE PID UPDATE LOGIC ---
    if (outputPWM_Right == 0.0 && targetRPM_Right > 0) { // Only initialize with base_pwm if we're trying to move
        outputPWM_Right = base_pwm_R;
    }
    
    // Incrementally adjust outputPWM_Right based on PID terms
    outputPWM_Right = outputPWM_Right + P_term_R + I_term_R + D_term_R;

    if (targetRPM_Right > 0 && outputPWM_Right > 0 && outputPWM_Right < MIN_EFFECTIVE_PWM_R) { outputPWM_Right = MIN_EFFECTIVE_PWM_R; }
    else if (targetRPM_Right == 0) { outputPWM_Right = 0; }

    if (outputPWM_Right > MAX_PWM_OVERALL) outputPWM_Right = MAX_PWM_OVERALL;
    if (outputPWM_Right < MIN_PWM_OVERALL) outputPWM_Right = MIN_PWM_OVERALL;
    prevError_Right = error_Right;


    // --- Apply PWM to BOTH Motors (Forward Direction) ---
    // Left Motor Control
    digitalWrite(IN1L, HIGH); // Left Motor Forward
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, (int)outputPWM_Left);

    // Right Motor Control
    digitalWrite(IN3R, HIGH); // Right Motor Forward
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, (int)outputPWM_Right);

    // --- Data Stream to Laptop (via Serial1 / HC-05) ---
    // Format: "Timestamp_ms,CurrentRPM_L,OutputPWM_L,CurrentRPM_R,OutputPWM_R"
    // This compact format is easy for Python to parse and plot.
    Serial1.print(currentTime);
    Serial1.print(",");
    Serial1.print(currentRPM_Left);
    Serial1.print(",");
    Serial1.print((int)outputPWM_Left);
    Serial1.print(",");
    Serial1.print(currentRPM_Right);
    Serial1.print(",");
    Serial1.println((int)outputPWM_Right); // println adds newline

    // --- Debug Print to Arduino IDE Serial Monitor (via USB) ---
    // This is for your visual debugging on the laptop connected via USB
    Serial.print("CRPM_L: "); Serial.print(currentRPM_Left);
    Serial.print(" | PWM_L: "); Serial.print((int)outputPWM_Left);
    Serial.print(" | CRPM_R: "); Serial.print(currentRPM_Right);
    Serial.print(" | PWM_R: "); Serial.println((int)outputPWM_Right);
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
