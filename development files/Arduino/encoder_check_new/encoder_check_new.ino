// --- Encoder Pins and Variables ---
const int LEFT_ENCODER_PIN = 3;  // Connect Left Encoder to Digital Pin 3 (Interrupt 1)
const int RIGHT_ENCODER_PIN = 2; // Connect Right Encoder to Digital Pin 2 (Interrupt 0)

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

const int ENAR = 10;  // PWM pin for Right Motor speed
const int IN3R = 5;   // Right Motor Direction Pin 1
const int IN4R = 4;   // Right Motor Direction Pin 2

const int ENAL = 9; // PWM pin for Left Motor speed
const int IN1L = 8;  // Left Motor Direction Pin 1
const int IN2L = 7;  // Left Motor Direction Pin 2

// Encoder properties
const int PPR = 8; // Pulses Per Revolution of your encoder
const unsigned long DEBOUNCE_DELAY_MS = 50; // Debounce for encoder interrupts
volatile unsigned long lastPulseTimeLeft = 0;
volatile unsigned long lastPulseTimeRight = 0;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP); // Use INPUT_PULLUP if encoder outputs open-collector/open-drain
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE); // Trigger on any state change
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW); analogWrite(ENAL, 90);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);
}

float MEASURE_TIME_MS = 1000;

void loop() {
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastPrintTime >= 1000) {  // 1000 ms = 1 second
    lastPrintTime = currentTime;

    noInterrupts();
    long currentLeftTicks_copy = leftEncoderTicks;
    long currentRightTicks_copy = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    Serial1.print("currentLeftTicks_copy: ");
    Serial1.print(currentLeftTicks_copy);
    Serial1.print(" | currentRightTicks_copy: ");
    Serial1.println(currentRightTicks_copy);

    float currentRPM_Left = (float)currentLeftTicks_copy / PPR / (MEASURE_TIME_MS / 60000.0);
    float currentRPM_Right = (float)currentRightTicks_copy / PPR / (MEASURE_TIME_MS / 60000.0);

    Serial1.print("currentRPM_Left: ");
    Serial1.print(currentRPM_Left);
    Serial1.print(" | currentRPM_Right: ");
    Serial1.println(currentRPM_Right);
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
