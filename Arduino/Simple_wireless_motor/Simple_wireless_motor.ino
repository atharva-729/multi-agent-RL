// Motor A (Left Side)
const int ENAL = 10;
const int IN1L = 8;
const int IN2L = 7;

// Motor B (Right Side)
const int ENAR = 9;
const int IN3R = 5;
const int IN4R = 4;

void setup() {
  // Motor pins
  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  // Start with motors off
  stopLeft();
  stopRight();

  // Serial setup
  Serial.begin(9600);     // For USB debugging
  Serial1.begin(9600);    // For HC-05 Bluetooth
}

void loop() {
  if (Serial1.available()) {
    char cmd = Serial1.read();

    switch (cmd) {
      case 'F': forwardBoth(); break;
      case 'B': backwardBoth(); break;
      case 'S': stopBoth(); break;
      case 'L': forwardLeft(); break;
      case 'l': backwardLeft(); break;
      case 'R': forwardRight(); break;
      case 'r': backwardRight(); break;
      case 'x': stopLeft(); break;
      case 'y': stopRight(); break;
    }

    // Optional debug
    Serial.print("Received: ");
    Serial.println(cmd);
  }
}

// --- Motor Control Functions ---

void forwardLeft() {
  digitalWrite(IN1L, HIGH);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 200);  // adjust speed as needed
}

void backwardLeft() {
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, HIGH);
  analogWrite(ENAL, 200);
}

void stopLeft() {
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  analogWrite(ENAL, 0);
}

void forwardRight() {
  digitalWrite(IN3R, HIGH);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 200);
}

void backwardRight() {
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, HIGH);
  analogWrite(ENAR, 200);
}

void stopRight() {
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  analogWrite(ENAR, 0);
}

void forwardBoth() {
  forwardLeft();
  forwardRight();
}

void backwardBoth() {
  backwardLeft();
  backwardRight();
}

void stopBoth() {
  stopLeft();
  stopRight();
}
