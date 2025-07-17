void setup() {
  Serial.begin(9600);     // For monitoring via USB
  Serial1.begin(9600);    // For HC-05 Bluetooth
  Serial.println("Arduino is ready.");
}

void loop() {
  // From HC-05 → to USB
  if (Serial1.available()) {
    String btData = Serial1.readStringUntil('\n');
    Serial.print("Received via BT: ");
    Serial.println(btData);

    // Respond back to Bluetooth
    Serial1.print("Echo: ");
    Serial1.println(btData);
  }

  // From USB Serial Monitor → to HC-05
  if (Serial.available()) {
    String pcData = Serial.readStringUntil('\n');
    Serial1.print("From PC: ");
    Serial1.println(pcData);
  }
}
