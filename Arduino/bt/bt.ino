void setup() {
  Serial1.begin(9600);
  pinMode(13, OUTPUT);  // Built-in LED
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    if (c == 'F') {
      digitalWrite(13, HIGH);
    }
    if (c == 'B') {
      digitalWrite(13, LOW);
    }
  }
}
