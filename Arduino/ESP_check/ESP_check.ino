#include <HardwareSerial.h> // Required for Serial1, Serial2, etc.

// Define which hardware serial port on the Mega is connected to the ESP-01
// On Arduino Mega, Serial1 uses Digital Pins 18 (TX1) and 19 (RX1)
#define ESP_SERIAL Serial1

// Baud rate for communication with the ESP-01 module.
// Newly flashed AT firmware usually defaults to 115200 baud.
const long ESP_BAUD_RATE = 115200; // Keep this at 115200 for your new firmware

void setup() {
  // Initialize communication with your computer via the USB Serial Monitor
  // This is where you'll type commands and see responses.
  Serial.begin(9600); // Standard baud rate for Arduino's USB Serial Monitor

  // Initialize communication with the ESP-01 module on Serial1
  // This baud rate must match the ESP-01's baud rate (115200 for new firmware)
  ESP_SERIAL.begin(ESP_BAUD_RATE);

  // Print some helpful messages to the Serial Monitor
  Serial.println("--- ESP-01 AT Command Passthrough Test ---");
  Serial.print("USB Serial Monitor Baud Rate: 9600 | ESP-01 Communication Baud Rate: ");
  Serial.println(ESP_BAUD_RATE);
  Serial.println("Type AT commands in the Serial Monitor input field and press Enter.");
  Serial.println("Example: Type 'AT' then press Enter.");
  Serial.println("------------------------------------------");
}

void loop() {
  // Check if any data is available from the ESP-01 (via Serial1)
  if (ESP_SERIAL.available()) {
    // Read the data byte by byte and send it to your computer's Serial Monitor
    Serial.write(ESP_SERIAL.read());
  }

  // Check if any data is available from your computer's Serial Monitor (what you type)
  if (Serial.available()) {
    // Read the data byte by byte and send it to the ESP-01 module (via Serial1)
    ESP_SERIAL.write(Serial.read());
  }
}