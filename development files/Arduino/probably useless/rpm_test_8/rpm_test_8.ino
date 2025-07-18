// --- Motor Control Pins ---
int in1 = 8; // Input 1 for Motor A direction
int in2 = 7; // Input 2 for Motor A direction

int in3 = 5; // Input 1 for Motor B direction (will be LOW)
int in4 = 4; // Input 2 for Motor B direction (will be LOW)

// --- TCRT5000 Sensor Pin (Digital Output D0 connected to Digital Pin 2) ---
const int TCRT5000_PIN = 3;

// --- Encoder Variables ---
volatile long pulseCount = 0;
// --- UPDATED FOR 8 BLACK STRIPES, EFFECTIVELY COUNTING 16 PULSES PER REVOLUTION ---
// With 8 black stripes, if the system is effectively counting both edges per stripe,
// or if the sensor output logic causes 2 pulses per stripe, then PPR = 16.
const int PULSES_PER_REVOLUTION = 16;
// --- END UPDATE ---
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI;

// --- Debouncing Variables ---
volatile unsigned long lastPulseTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 20; // Adjust this value (e.g., 5ms to 50ms) if needed for stable readings

// --- Timing Variables for RPM Calculation ---
unsigned long lastUpdateTime = 0;
const unsigned long RPM_CALC_INTERVAL = 1000; // Calculate RPM every 1 second (1000 milliseconds)

// --- Timing Variables for Raw Sensor Printing ---
unsigned long lastRawPrintTime = 0;
const unsigned long RAW_PRINT_INTERVAL_MS = 20; // Print raw value every 20 milliseconds (50 times per second)

// --- Interrupt Service Routine (ISR) ---
// This function runs automatically whenever the TCRT5000_PIN changes state (RISING edge).
void countEncoderPulses() {
  unsigned long currentTime = millis();
  // Check if enough time has passed since the last valid pulse (debouncing)
  if (currentTime - lastPulseTime > DEBOUNCE_DELAY_MS) {
    pulseCount++;              // Increment pulse count
    lastPulseTime = currentTime; // Update the time of the last VALID pulse
  }
}

void setup() {
  // --- Motor Setup ---
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // --- Sensor Setup ---
  pinMode(TCRT5000_PIN, INPUT);
  // Keep RISING mode. The PPR adjustment handles the effective doubling of pulses.
  attachInterrupt(digitalPinToInterrupt(TCRT5000_PIN), countEncoderPulses, CHANGE);

  // --- Serial Communication ---
  Serial.begin(9600); // Baud rate for Serial Monitor
  Serial.println("Robot Test: Left Motor FORWARD & Encoder Readings for 8 Stripes");
  Serial.println("PPR is now set to 16 (to correct for double counting).");
  Serial.println("------------------------------------------------------------------");

  // --- Start Left Motor (Motor A) Forward ---
  // digitalWrite(in1, LOW); // Motor A forward
  // digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  // Initialize timers
  lastUpdateTime = millis();
  lastPulseTime = millis();
  lastRawPrintTime = millis();
}

void loop() {
  // --- Limit Raw Sensor Value Printing ---
  if (millis() - lastRawPrintTime >= RAW_PRINT_INTERVAL_MS) {
    int rawSensorValue = digitalRead(TCRT5000_PIN);
    // Serial.print("Raw: ");
    // Serial.println(rawSensorValue);
    lastRawPrintTime = millis();
  }

  // --- Encoder Data Processing and Output ---
  if (millis() - lastUpdateTime >= RPM_CALC_INTERVAL) {
    noInterrupts();
    long currentPulseCount = pulseCount;
    pulseCount = 0;
    interrupts();

    float revolutionsInInterval = (float)currentPulseCount / PULSES_PER_REVOLUTION;
    float rpm = (revolutionsInInterval / (RPM_CALC_INTERVAL / 1000.0)) * 60.0;
    float distanceTraveled_mm = revolutionsInInterval * WHEEL_CIRCUMFERENCE_MM;

    Serial.print("   --- CALCULATED --- RPM: ");
    Serial.print(rpm);
    Serial.print(" | Pulses: ");
    Serial.print(currentPulseCount);
    Serial.print(" | Dist: ");
    Serial.print(distanceTraveled_mm);
    Serial.println(" mm");

    lastUpdateTime = millis();
  }
}