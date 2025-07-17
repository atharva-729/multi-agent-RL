// --- Motor Driver Pins (L298N) ---
const int ENAR = 10; // PWM pin for Right Motor speed (Connect to L298N ENB)
const int IN3R = 5;  // Right Motor Direction Pin 1 (Connect to L298N IN3)
const int IN4R = 4;  // Right Motor Direction Pin 2 (Connect to L298N IN4)

// --- Encoder Pins and Variables ---
const int RIGHT_ENCODER_PIN = 3; // Connect Right Encoder to Digital Pin 3 (Interrupt 1)
volatile long rightEncoderTicks = 0;

// Encoder properties
const int PPR = 16; // Pulses Per Revolution for your encoder
const unsigned long DEBOUNCE_DELAY_MS = 20; // Debounce for encoder pulses
volatile unsigned long lastPulseTimeRight = 0;

// --- PWM Sweep Settings ---
const int PWM_START = 0;   // Starting PWM value
const int PWM_END = 255;   // Ending PWM value
const int PWM_STEP = 5;    // Increment PWM by this value each step

// --- Measurement Settings ---
// Time to let motor stabilize before measuring RPM (ms)
const unsigned long STABILIZE_TIME_MS = 2000;
// Time to measure RPM after stabilization (ms)
const unsigned long MEASURE_TIME_MS = 1000;

void setup() {
  Serial.begin(9600);
  Serial.println("PWM,RPM"); // Header for CSV data
  Serial.println("Starting Motor Characterization...");

  // Motor Driver Pin Setup
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder Pin Setup
  pinMode(RIGHT_ENCODER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  // Ensure motor is off initially
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
  delay(1000);
}

void loop() {
  // Loop through different PWM values
  for (int currentPWM = PWM_START; currentPWM <= PWM_END; currentPWM += PWM_STEP) {
    // 1. Apply PWM and set direction (Forward for right motor)
    digitalWrite(IN3R, HIGH); // Adjust for your motor's forward direction
    digitalWrite(IN4R, LOW);
    analogWrite(ENAR, currentPWM);

    Serial.print("Applying PWM: ");
    Serial.println(currentPWM);

    // 2. Wait for motor speed to stabilize
    delay(STABILIZE_TIME_MS);

    // 3. Measure RPM for a fixed duration
    noInterrupts();
    rightEncoderTicks = 0; // Reset ticks before measurement
    interrupts();

    unsigned long startTime = millis();
    while (millis() - startTime < MEASURE_TIME_MS) {
      // Just wait and let interrupts count pulses
    }

    noInterrupts();
    long pulsesInMeasureTime = rightEncoderTicks;
    interrupts();

    // 4. Calculate RPM
    // RPM = (pulses / PPR) / (MEASURE_TIME_MS / 60000.0)
    float currentRPM = (float)pulsesInMeasureTime / PPR / (MEASURE_TIME_MS / 60000.0);

    // 5. Send data over Serial (CSV format)
    Serial.print(currentPWM);
    Serial.print(",");
    Serial.println(currentRPM);

    // Add a small delay between steps to allow serial buffer to clear and for observation
    delay(100);
  }

  // After sweeping all PWM values, stop the motor
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
  Serial.println("Characterization Complete. Motor Stopped.");

  // Keep the loop from repeating constantly; you can reset Arduino to run again
  while(true);
}

// --- Encoder Interrupt Service Routine (ISR) ---
void rightEncoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    rightEncoderTicks++;
    lastPulseTimeRight = currentTime;
  }
}