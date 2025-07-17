// ------------------- Pin Definitions -------------------
const int ENAL = 9;  // Left Motor PWM
const int IN1L = 8;
const int IN2L = 7;

const int ENAR = 10; // Right Motor PWM
const int IN3R = 5;
const int IN4R = 4;

const int LEFT_ENCODER_PIN = 2;
const int RIGHT_ENCODER_PIN = 3;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

// ------------------- Encoder Settings -------------------
const int PPR = 16;  // Pulses per wheel revolution (adjust as needed)
const float WHEEL_CIRC_CM = 7.0;
const float CM_PER_TICK = WHEEL_CIRC_CM / PPR;

// ------------------- PID + Feedforward -------------------
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float errorL = 0, prevErrorL = 0, integralL = 0;
float errorR = 0, prevErrorR = 0, integralR = 0;

const int LOOP_INTERVAL_MS = 100;
const int MIN_PWM = 65;
const int MAX_PWM = 255;

const float REGRESSION_SLOPE_L = 0.53;
const float REGRESSION_INTERCEPT_L = -0.162;

const float REGRESSION_SLOPE_R = 0.526;
const float REGRESSION_INTERCEPT_R = -1.518;

float pwmL = 0;
float pwmR = 0;

String btInput = ""; // Buffer for incoming Bluetooth command

// ------------------- Target Distance -------------------
const float TARGET_DISTANCE_CM = 100.0;
const long TARGET_TICKS = TARGET_DISTANCE_CM / CM_PER_TICK;

unsigned long lastLoopTime = 0;

void setup() {
  Serial1.begin(9600);     // USB debugging
  Serial1.begin(9600);    // Bluetooth (HC-05)

  pinMode(ENAL, OUTPUT); pinMode(IN1L, OUTPUT); pinMode(IN2L, OUTPUT);
  pinMode(ENAR, OUTPUT); pinMode(IN3R, OUTPUT); pinMode(IN4R, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

  stopMotors();
  delay(15000); // time for bluetooth to get ready

  Serial1.println("Bluetooth ready. Send KP/KI/KD commands.");
  Serial1.print("Target Distance (cm): "); Serial1.println(TARGET_DISTANCE_CM);
  Serial1.print("Target Ticks: "); Serial1.println(TARGET_TICKS);

  lastLoopTime = millis();
}

void loop() {
  // --- Check for PID gain updates via Bluetooth ---
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      processBTCommand(btInput);
      btInput = "";
    } else {
      btInput += c;
    }
  }

  // --- PID Loop ---
  unsigned long now = millis();
  if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = now;

    long leftTicks = leftEncoderTicks;
    long rightTicks = rightEncoderTicks;

    float leftRPM = (leftTicks / (float)PPR) * (60000.0 / LOOP_INTERVAL_MS);
    float rightRPM = (rightTicks / (float)PPR) * (60000.0 / LOOP_INTERVAL_MS);

    long remainingL = TARGET_TICKS - leftTicks;
    long remainingR = TARGET_TICKS - rightTicks;

    float targetRPM_L = (remainingL > 0) ? 60 : -60;
    float targetRPM_R = (remainingR > 0) ? 60 : -60;

    float base_pwm_L = (abs(targetRPM_L) - REGRESSION_INTERCEPT_L) / REGRESSION_SLOPE_L;
    float base_pwm_R = (abs(targetRPM_R) - REGRESSION_INTERCEPT_R) / REGRESSION_SLOPE_R;

    // Left motor PID
    errorL = targetRPM_L - leftRPM;
    integralL += errorL * (LOOP_INTERVAL_MS / 1000.0);
    float derivativeL = (errorL - prevErrorL) / (LOOP_INTERVAL_MS / 1000.0);
    pwmL = base_pwm_L + Kp * errorL + Ki * integralL + Kd * derivativeL;

    // Right motor PID
    errorR = targetRPM_R - rightRPM;
    integralR += errorR * (LOOP_INTERVAL_MS / 1000.0);
    float derivativeR = (errorR - prevErrorR) / (LOOP_INTERVAL_MS / 1000.0);
    pwmR = base_pwm_R + Kp * errorR + Ki * integralR + Kd * derivativeR;

    prevErrorL = errorL;
    prevErrorR = errorR;

    // Clamp and apply
    int outL = constrain((int)pwmL, MIN_PWM, MAX_PWM);
    int outR = constrain((int)pwmR, MIN_PWM, MAX_PWM);

    applyMotor(outL, remainingL > 0, 'L');
    applyMotor(outR, remainingR > 0, 'R');

    Serial1.print("Ticks L/R: "); Serial1.print(leftTicks); Serial1.print("/"); Serial1.print(rightTicks);
    Serial1.print(" | PWM L/R: "); Serial1.print(outL); Serial1.print("/"); Serial1.println(outR);
    Serial1.print("PWM L/R: "); Serial1.print(outL); Serial1.print("/"); Serial1.println(outR);

    // Stop when close
    if (abs(remainingL) < 3 && abs(remainingR) < 3) {
      stopMotors();
      Serial1.println("Target Reached.");
      Serial1.println("Target Reached.");
      while (true); // Freeze
    }

    noInterrupts();
    leftEncoderTicks = 0;
    rightEncoderTicks = 0;
    interrupts();
  }
}

void applyMotor(int pwm, bool forward, char side) {
  if (side == 'L') {
    digitalWrite(IN1L, forward ? HIGH : LOW);
    digitalWrite(IN2L, forward ? LOW : HIGH);
    analogWrite(ENAL, pwm);
  } else {
    digitalWrite(IN3R, forward ? HIGH : LOW);
    digitalWrite(IN4R, forward ? LOW : HIGH);
    analogWrite(ENAR, pwm);
  }
}

void stopMotors() {
  analogWrite(ENAL, 0); digitalWrite(IN1L, LOW); digitalWrite(IN2L, LOW);
  analogWrite(ENAR, 0); digitalWrite(IN3R, LOW); digitalWrite(IN4R, LOW);
}

// --- Encoder ISRs ---
void leftEncoderISR() { leftEncoderTicks++; }
void rightEncoderISR() { rightEncoderTicks++; }

// --- Bluetooth Command Parser ---
void processBTCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("KP ")) {
    Kp = cmd.substring(3).toFloat();
    Serial1.print("Kp updated to "); Serial1.println(Kp);
  } else if (cmd.startsWith("KI ")) {
    Ki = cmd.substring(3).toFloat();
    Serial1.print("Ki updated to "); Serial1.println(Ki);
  } else if (cmd.startsWith("KD ")) {
    Kd = cmd.substring(3).toFloat();
    Serial1.print("Kd updated to "); Serial1.println(Kd);
  } else {
    Serial1.print("Invalid cmd: "); Serial1.println(cmd);
  }
}
