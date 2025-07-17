// --- Motor Driver Pins ---
const int ENAL = 9;
const int IN1L = 8;
const int IN2L = 7;

const int ENAR = 10;
const int IN3R = 5;
const int IN4R = 4;

// --- Encoder Pins (Analog Read) ---
const int LEFT_ANALOG_PIN = A1;
const int RIGHT_ANALOG_PIN = A0;
const int THRESHOLD = 500;

// --- Encoder properties ---
const int PPR = 8;

// --- State variables ---
int prevLeft = 0;
int prevRight = 0;
long pulsesLeft = 0;
long pulsesRight = 0;
unsigned long lastRPMTime = 0;

void setup() {
  Serial1.begin(9600);

  // Motor setup
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  // Encoder input
  pinMode(LEFT_ANALOG_PIN, INPUT);
  pinMode(RIGHT_ANALOG_PIN, INPUT);
  delay(15000);

  // Start motors (PWM = 120, forward)
  digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 120);

  digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 120);

}

void loop() {
  int leftVal = analogRead(LEFT_ANALOG_PIN);
  int rightVal = analogRead(RIGHT_ANALOG_PIN);

  // Rising edge detection
  if (leftVal > THRESHOLD && prevLeft <= THRESHOLD) pulsesLeft++;
  if (rightVal > THRESHOLD && prevRight <= THRESHOLD) pulsesRight++;

  prevLeft = leftVal;
  prevRight = rightVal;

  // Print RPM every 1000 ms
  if (millis() - lastRPMTime >= 1000) {
    float rpmLeft = (float)pulsesLeft / PPR * 60.0;
    float rpmRight = (float)pulsesRight / PPR * 60.0;

    Serial1.print("RPM_Left: ");
    Serial1.print(rpmLeft);
    Serial1.print(" | RPM_Right: ");
    Serial1.println(rpmRight);

    pulsesLeft = 0;
    pulsesRight = 0;
    lastRPMTime = millis();
  }
}
