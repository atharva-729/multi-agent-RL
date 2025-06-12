// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 9; // PWM pin for Left Motor speed
const int IN1L = 4;  // Left Motor Direction Pin 1
const int IN2L = 5;  // Left Motor Direction Pin 2

// Right Motor pins
const int ENAR = 10;  // PWM pin for Right Motor speed
const int IN3R = 7;  // Right Motor Direction Pin 1
const int IN4R = 8;  // Right Motor Direction Pin 2

// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;  // Ticks for the LEFT encoder
volatile long rightEncoderTicks = 0; // Ticks for the RIGHT encoder

// Encoder properties
const int PPR = 16; // Pulses Per Revolution for your encoder
const unsigned long DEBOUNCE_DELAY_MS = 20; // Debounce for encoder pulses
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

// --- PWM Sweep Settings ---
const int PWM_START = 0;   // Starting PWM value
const int PWM_END = 255;   // Ending PWM value
const int PWM_STEP = 5;    // Increment PWM by this value each step

// --- Measurement Settings ---
// Time to let motor stabilize before measuring RPM (ms)
const unsigned long STABILIZE_TIME_MS = 500;
// Time to measure RPM after stabilization (ms)
const unsigned long MEASURE_TIME_MS = 300; // Measure for 1 second

void setup() {
  Serial.begin(9600);
  Serial.println("PWM,RPM_Left,RPM_Right"); // Header for CSV data
  Serial.println("Starting Dual Motor Characterization...");

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

  // Ensure both motors are off initially
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0);

  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
  delay(1000);
}

void loop() {
  // Loop through different PWM values
  Serial.println("PWM,RPM_Left,RPM_Right");
  
  for (int currentPWM = PWM_START; currentPWM <= PWM_END; currentPWM += PWM_STEP) {
    // 1. Apply PWM and set direction for BOTH motors (Forward)
    digitalWrite(IN1L, HIGH); // Left Motor Forward
    digitalWrite(IN2L, LOW);
    analogWrite(ENAL, currentPWM);

    digitalWrite(IN3R, HIGH); // Right Motor Forward
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, currentPWM);

    Serial.print("Applying PWM: ");
    Serial.println(currentPWM);

    // 2. Wait for motor speeds to stabilize
    delay(STABILIZE_TIME_MS);

    // 3. Measure RPM for a fixed duration for BOTH motors
    noInterrupts();
    leftEncoderTicks = 0;  // Reset ticks before measurement
    rightEncoderTicks = 0; // Reset ticks before measurement
    interrupts();

    unsigned long startTime = millis();
    while (millis() - startTime < MEASURE_TIME_MS) {
      // Just wait and let interrupts count pulses
    }

    noInterrupts();
    long pulsesLeft = leftEncoderTicks;
    long pulsesRight = rightEncoderTicks;
    interrupts();

    // 4. Calculate RPM for BOTH motors
    float currentRPM_Left = (float)pulsesLeft / PPR / (MEASURE_TIME_MS / 60000.0);
    float currentRPM_Right = (float)pulsesRight / PPR / (MEASURE_TIME_MS / 60000.0);

    // 5. Send data over Serial (CSV format)
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.print(currentRPM_Left);
    Serial.print(",");
    Serial.println(currentRPM_Right);

    // Add a small delay between steps to allow serial buffer to clear and for observation
    delay(100);
  }

  // After sweeping all PWM values, stop both motors
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0);

  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
  Serial.println("Characterization Complete. Motors Stopped.");

  // Keep the loop from repeating constantly; you can reset Arduino to run again
  while(true);
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