void setup() {
  Serial.begin(9600);    // USB (PC ↔ Arduino IDE Serial Monitor)
  Serial1.begin(9600);   // HC-05 (Bluetooth)
}

void loop() {
  // USB to Bluetooth
  if (Serial.available()) {
    char c = Serial.read();
    Serial1.write(c);  // Send to HC-05
  }

  // Bluetooth to USB
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);  // Show in Serial Monitor
  }
}
