// --- Motor Pins ---
const int ENAL = 9;
const int IN1L = 8;
const int IN2L = 7;

const int ENAR = 10;
const int IN3R = 5;
const int IN4R = 4;

// --- Encoder Pins ---
const int LEFT_ENCODER_PIN = 2;
const int RIGHT_ENCODER_PIN = 3;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

const int PPR = 16; // Pulses per revolution

// --- Timing ---
const unsigned long LOOP_INTERVAL_MS = 100;
unsigned long lastLoopTime = 0;

// --- PID Variables ---
float targetRPM_L = 80.0;
float targetRPM_R = 80.0;

float Kp_L = 0.8, Ki_L = 0.0, Kd_L = 0.0;
float Kp_R = 0.8, Ki_R = 0.0, Kd_R = 0.0;

float errorL = 0, prevErrorL = 0, integralL = 0;
float errorR = 0, prevErrorR = 0, integralR = 0;

const int MIN_PWM = 60;
const int MAX_PWM = 255;

float pwmL = 0;
float pwmR = 0;

void setup() {
  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  Serial1.begin(9600); // Bluetooth
  delay(10000);
  Serial1.println("Velocity PID control active.");
  lastLoopTime = millis();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = now;

    long ticksL, ticksR;
    noInterrupts();
    ticksL = leftEncoderTicks;
    ticksR = rightEncoderTicks;
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();

    float rpmL = (ticksL / (float)PPR) * (60000.0 / LOOP_INTERVAL_MS);
    float rpmR = (ticksR / (float)PPR) * (60000.0 / LOOP_INTERVAL_MS);

    // Left wheel PID
    errorL = targetRPM_L - rpmL;
    integralL += errorL * (LOOP_INTERVAL_MS / 1000.0);
    float derivativeL = (errorL - prevErrorL) / (LOOP_INTERVAL_MS / 1000.0);
    pwmL += Kp_L * errorL + Ki_L * integralL + Kd_L * derivativeL;
    prevErrorL = errorL;

    // Right wheel PID
    errorR = targetRPM_R - rpmR;
    integralR += errorR * (LOOP_INTERVAL_MS / 1000.0);
    float derivativeR = (errorR - prevErrorR) / (LOOP_INTERVAL_MS / 1000.0);
    pwmR += Kp_R * errorR + Ki_R * integralR + Kd_R * derivativeR;
    prevErrorR = errorR;

    int outL = constrain((int)pwmL, MIN_PWM, MAX_PWM);
    int outR = constrain((int)pwmR, MIN_PWM, MAX_PWM);

    // Apply motor commands
    digitalWrite(IN1L, HIGH); digitalWrite(IN2L, LOW);
    analogWrite(ENAL, outL);

    digitalWrite(IN3R, HIGH); digitalWrite(IN4R, LOW);
    analogWrite(ENAR, outR);

    Serial1.print("RPM_L: "); Serial1.print(rpmL);
    Serial1.print(" PWM_L: "); Serial1.print(outL);
    Serial1.print(" | RPM_R: "); Serial1.print(rpmR);
    Serial1.print(" PWM_R: "); Serial1.println(outR);
  }
}

void leftEncoderISR() { leftEncoderTicks++; }
void rightEncoderISR() { rightEncoderTicks++; }
