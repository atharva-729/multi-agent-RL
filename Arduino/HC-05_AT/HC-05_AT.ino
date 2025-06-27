void setup() {
  Serial.begin(9600);     // USB Serial Monitor
  Serial1.begin(38400);   // HC-05 default AT baud rate
  Serial.println("Enter AT commands:");
}

void loop() {
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
}
