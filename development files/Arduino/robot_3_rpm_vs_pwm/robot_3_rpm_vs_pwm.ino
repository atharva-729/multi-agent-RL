// --- Motor Driver Pins (L298N) ---
// Left Motor pins
const int ENAL = 9; // PWM pin for Left Motor speed
const int IN1L = 8;  // Left Motor Direction Pin 1
const int IN2L = 7;  // Left Motor Direction Pin 2

// Right Motor pins
const int ENAR = 10;  // PWM pin for Right Motor speed
const int IN3R = 5;   // Right Motor Direction Pin 1
const int IN4R = 4;   // Right Motor Direction Pin 2

// --- Analog Encoder Pins ---
const int LEFT_ANALOG_PIN = A1;
const int RIGHT_ANALOG_PIN = A0;
const int THRESHOLD = 500; // Analog threshold to detect pulse

// --- Encoder Properties ---
const int PPR = 8; // Pulses Per Revolution for your encoder

// --- PWM Sweep Settings ---
const int PWM_START = 0;
const int PWM_END = 255;
const int PWM_STEP = 5;

// --- Measurement Settings ---
const unsigned long STABILIZE_TIME_MS = 500;
const unsigned long MEASURE_TIME_MS = 300;

void setup() {
  Serial1.begin(9600);
  Serial1.println("PWM,RPM_Left,RPM_Right");
  Serial1.println("Starting Dual Motor Characterization...");

  // Motor Driver Pin Setup
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Encoder analog pins
  pinMode(LEFT_ANALOG_PIN, INPUT);
  pinMode(RIGHT_ANALOG_PIN, INPUT);

  // Stop motors initially
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);

  delay(2000); // Wait before starting
}

void loop() {
  Serial1.println("PWM,RPM_Left,RPM_Right");

  for (int currentPWM = PWM_START; currentPWM <= PWM_END; currentPWM += PWM_STEP) {
    // 1. Set motor direction and apply PWM
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
    analogWrite(ENAL, currentPWM);

    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
    analogWrite(ENAR, currentPWM);

    Serial1.print("Applying PWM: ");
    Serial1.println(currentPWM);

    delay(STABILIZE_TIME_MS);

    // 2. Count pulses via analogRead (rising edge detection)
    long pulsesLeft = 0;
    long pulsesRight = 0;
    int prevLeft = 0;
    int prevRight = 0;

    unsigned long startTime = millis();
    while (millis() - startTime < MEASURE_TIME_MS) {
      int leftVal = analogRead(LEFT_ANALOG_PIN);
      int rightVal = analogRead(RIGHT_ANALOG_PIN);

      if (leftVal > THRESHOLD && prevLeft <= THRESHOLD) {
        pulsesLeft++;
      }
      if (rightVal > THRESHOLD && prevRight <= THRESHOLD) {
        pulsesRight++;
      }

      prevLeft = leftVal;
      prevRight = rightVal;
    }

    // 3. Compute RPMs
    float currentRPM_Left = (float)pulsesLeft / PPR / (MEASURE_TIME_MS / 60000.0);
    float currentRPM_Right = (float)pulsesRight / PPR / (MEASURE_TIME_MS / 60000.0);

    // 4. Output result
    Serial1.print(currentPWM);
    Serial1.print(",");
    Serial1.print(currentRPM_Left);
    Serial1.print(",");
    Serial1.println(currentRPM_Right);

    delay(100); // Allow serial buffer to clear
  }

  // 5. Stop motors after full sweep
  digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW); analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW); analogWrite(ENAR, 0);

  Serial1.println("Characterization Complete. Motors Stopped.");

  // Prevent repeat execution
  while (true);
}
