String inputString = "";
bool newData = false;

// Fake state
int x_pos = 10, y_pos = 5;
bool left = false, center = true, right = false;

void setup() {
  Serial1.begin(9600);
  inputString.reserve(50);
}

void loop() {
  recvWithEndMarker();  // Always check if new data has come
  if (newData) {
    Serial1.print("New Target Received: ");
    Serial1.println(inputString);
    // Here you can parse inputString to extract coords if needed
    inputString = "";
    newData = false;
  }

  // Send sensor and position info periodically
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 500) {
    Serial1.print("POS:");
    Serial1.print(x_pos);
    Serial1.print(",");
    Serial1.print(y_pos);
    Serial1.print(" | OBST:");
    if (left) Serial1.print("L ");
    if (center) Serial1.print("C ");
    if (right) Serial1.print("R ");
    if (!left && !center && !right) Serial1.print("None");
    Serial1.println();
    lastSendTime = millis();
  }
}

void recvWithEndMarker() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\n') {
      newData = true;
      break;
    }
    inputString += c;
  }
}
