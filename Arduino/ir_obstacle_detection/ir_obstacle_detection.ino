// Pin mapping
const int IR_LEFT = 30;
const int IR_CENTER = 50;
const int IR_RIGHT = 32;

// State storage to detect new obstacle appearance
bool prev_left = false;
bool prev_center = false;
bool prev_right = false;

void setup() {
  Serial.begin(9600);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

void loop() {
  // Read current sensor states
  bool left = digitalRead(IR_LEFT) == LOW;
  bool center = digitalRead(IR_CENTER) == LOW;
  bool right = digitalRead(IR_RIGHT) == LOW;

  // Only print when something changes
  if (left != prev_left || center != prev_center || right != prev_right) {
    Serial.print("Obstacle: ");
    if (left) Serial.print("L ");
    if (center) Serial.print("C ");
    if (right) Serial.print("R ");
    if (!left && !center && !right) Serial.print("None");
    Serial.println();

    // Update previous state
    prev_left = left;
    prev_center = center;
    prev_right = right;
  }

  delay(100);  // Reduce CPU usage and debounce sensor transitions
}
