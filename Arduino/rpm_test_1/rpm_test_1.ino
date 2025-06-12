// --- Motor Control Pins ---
int in1 = 8;
int in2 = 7;
int in3 = 5;
int in4 = 4;

// --- TCRT5000 Sensor Pin (Digital Output D0 connected to Digital Pin 2) ---
const int TCRT5000_PIN = 2;

// --- Encoder Variables ---
volatile long pulseCount = 0;
// --- CHANGED PPR HERE ---
// If you have 4 black segments, and you expect 4 pulses per revolution,
// but the readings are consistently double, then set PPR to 8.
const int PULSES_PER_REVOLUTION = 8; // Assuming 4 actual pulses, but effectively 8 are being counted
// --- END CHANGED PPR ---
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI;

// --- Debouncing Variables ---
volatile unsigned long lastPulseTime = 0;
const unsigned long DEBOUNCE_DELAY_MS = 20; // Keep this, adjust if needed

// --- Timing Variables for RPM Calculation ---
unsigned long lastUpdateTime = 0;
const unsigned long RPM_CALC_INTERVAL = 1000; // Calculate RPM every 1 second (1000 milliseconds)

// --- Timing Variables for Raw Sensor Printing ---
unsigned long lastRawPrintTime = 0;
const unsigned long RAW_PRINT_INTERVAL_MS = 20; // Print raw value every 20 milliseconds

// --- Interrupt Service Routine (ISR) ---
void countEncoderPulses() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTime > DEBOUNCE_DELAY_MS) {
    pulseCount++;
    lastPulseTime = currentTime;
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
  // Keep RISING for consistency, the PPR adjustment handles the effective doubling
  attachInterrupt(digitalPinToInterrupt(TCRT5000_PIN), countEncoderPulses, RISING);

  // --- Serial Communication ---
  Serial.begin(9600);
  Serial.println("Robot Test: Left Motor FORWARD & Encoder Readings");
  Serial.println("PPR is now set to 8 for 4 effective segments.");
  Serial.println("------------------------------------------------------------------");

  // --- Start Left Motor (Motor A) Forward ---
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

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