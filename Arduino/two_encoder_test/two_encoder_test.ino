// for understanding/debugging this code: go to intern project alt chat and just copy paste the code in ctrl+f

// --- Motor Control Pins ---
// Motor A connections (Left Side Motor)
int in1 = 8; // Input 1 for Motor A direction
int in2 = 7; // Input 2 for Motor A direction

// Motor B connections (Right Side Motor)
int in3 = 5; // Input 1 for Motor B direction
int in4 = 4; // Input 2 for Motor B direction

// --- TCRT5000 Sensor Pins ---
// Left Encoder: Digital Output D0 connected to Arduino Digital Pin 2
const int TCRT5000_PIN_LEFT = 2;
// Right Encoder: Digital Output D0 connected to Arduino Digital Pin 3
const int TCRT5000_PIN_RIGHT = 3;

// --- Encoder Variables (Left Side) ---
volatile long pulseCountLeft = 0;
const int PULSES_PER_REVOLUTION = 16; // 8 black stripes * 2 effective pulses/stripe = 16 PPR
const float WHEEL_DIAMETER_MM = 70.0;
const float WHEEL_CIRCUMFERENCE_MM = WHEEL_DIAMETER_MM * PI;

// --- Encoder Variables (Right Side) ---
volatile long pulseCountRight = 0;
// Assuming the right wheel has the same encoder setup and PPR
const int PULSES_PER_REVOLUTION_RIGHT = 16;

// --- Debouncing Variables ---
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;
const unsigned long DEBOUNCE_DELAY_MS = 20; // Adjust if needed for stable readings

// --- Timing Variables for RPM Calculation ---
unsigned long lastUpdateTime = 0;
const unsigned long RPM_CALC_INTERVAL = 1000; // Calculate RPM every 1 second (1000 milliseconds)

// --- Timing Variables for Raw Sensor Printing ---
unsigned long lastRawPrintTime = 0;
const unsigned long RAW_PRINT_INTERVAL_MS = 20; // Print raw value every 20 milliseconds (50 times per second)

// --- Interrupt Service Routines (ISRs) ---
// ISR for Left Encoder
void countEncoderPulsesLeft() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeLeft > DEBOUNCE_DELAY_MS) {
    pulseCountLeft++;
    lastPulseTimeLeft = currentTime;
  }
}

// ISR for Right Encoder
void countEncoderPulsesRight() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTimeRight > DEBOUNCE_DELAY_MS) {
    pulseCountRight++;
    lastPulseTimeRight = currentTime;
  }
}

void setup() {
  // --- Motor Setup ---
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off all motors initially
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // --- Sensor Setup ---
  pinMode(TCRT5000_PIN_LEFT, INPUT);
  pinMode(TCRT5000_PIN_RIGHT, INPUT);

  // Attach interrupts for both encoders
  // Digital Pin 2 -> Interrupt 0 on Mega
  attachInterrupt(digitalPinToInterrupt(TCRT5000_PIN_LEFT), countEncoderPulsesLeft, RISING);
  // Digital Pin 3 -> Interrupt 1 on Mega
  attachInterrupt(digitalPinToInterrupt(TCRT5000_PIN_RIGHT), countEncoderPulsesRight, RISING);

  // --- Serial Communication ---
  Serial.begin(9600); // Baud rate for Serial Monitor
  Serial.println("Robot Test: Dual Encoder Readings");
  // --- UPDATED SERIAL PRINT ---
  Serial.println("Both Motors FORWARD.");
  // --- END UPDATE ---
  Serial.println("PPR for both sides is 16.");
  Serial.println("------------------------------------------------------------------");

  // --- Start Both Motors FORWARD ---
  // --- UPDATED MOTOR CONTROL ---
  digitalWrite(in1, LOW); // Motor A (Left) forward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);  // Motor B (Right) forward
  digitalWrite(in4, HIGH);
  // --- END UPDATE ---

  // Initialize timers
  lastUpdateTime = millis();
  lastPulseTimeLeft = millis();
  lastPulseTimeRight = millis();
  lastRawPrintTime = millis();
}

void loop() {
  // --- Limit Raw Sensor Value Printing ---
  if (millis() - lastRawPrintTime >= RAW_PRINT_INTERVAL_MS) {
    int rawSensorValueLeft = digitalRead(TCRT5000_PIN_LEFT);
    int rawSensorValueRight = digitalRead(TCRT5000_PIN_RIGHT);

    // Serial.print("Raw L: ");
    // Serial.print(rawSensorValueLeft);
    // Serial.print(" | Raw R: ");
    // Serial.println(rawSensorValueRight);
    lastRawPrintTime = millis(); // Reset timer for next raw print
  }

  // --- Encoder Data Processing and Output ---
  if (millis() - lastUpdateTime >= RPM_CALC_INTERVAL) {
    // --- Left Side Calculations ---
    noInterrupts(); // Disable interrupts for safe reading of volatile variables
    long currentPulseCountLeft = pulseCountLeft;
    pulseCountLeft = 0; // Reset for the next interval's count
    interrupts();       // Re-enable interrupts

    float revolutionsInIntervalLeft = (float)currentPulseCountLeft / PULSES_PER_REVOLUTION;
    float rpmLeft = (revolutionsInIntervalLeft / (RPM_CALC_INTERVAL / 1000.0)) * 60.0;
    float distanceTraveled_mm_Left = revolutionsInIntervalLeft * WHEEL_CIRCUMFERENCE_MM;

    // --- Right Side Calculations ---
    noInterrupts(); // Disable interrupts again for the right side
    long currentPulseCountRight = pulseCountRight;
    pulseCountRight = 0; // Reset for the next interval's count
    interrupts();       // Re-enable interrupts

    float revolutionsInIntervalRight = (float)currentPulseCountRight / PULSES_PER_REVOLUTION_RIGHT;
    float rpmRight = (revolutionsInIntervalRight / (RPM_CALC_INTERVAL / 1000.0)) * 60.0;
    float distanceTraveled_mm_Right = revolutionsInIntervalRight * WHEEL_CIRCUMFERENCE_MM; // Assuming same wheel size

    // --- Print Calculated Values ---
    Serial.print("   --- L --- RPM: ");
    Serial.print(rpmLeft);
    Serial.print(" | Pulses: ");
    Serial.print(currentPulseCountLeft);
    Serial.print(" | Dist: ");
    Serial.print(distanceTraveled_mm_Left);
    Serial.print(" mm");

    Serial.print("   --- R --- RPM: ");
    Serial.print(rpmRight);
    Serial.print(" | Pulses: ");
    Serial.print(currentPulseCountRight);
    Serial.print(" | Dist: ");
    Serial.print(distanceTraveled_mm_Right);
    Serial.println(" mm");

    lastUpdateTime = millis(); // Reset timer for the next RPM calculation interval
  }
}