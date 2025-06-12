#include <HardwareSerial.h> // Required for Serial1

// --- Motor Driver Pins (L298N) ---
const int ENAL = 10; // PWM pin for Left Motor speed
const int IN1L = 8;  // Left Motor Direction Pin 1
const int IN2L = 7;  // Left Motor Direction Pin 2

const int ENAR = 9;  // PWM pin for Right Motor speed
const int IN3R = 5;  // Right Motor Direction Pin 1
const int IN4R = 4;  // Right Motor Direction Pin 2

// --- PWM Output Limits ---
const int MIN_PWM = 0;
const int MAX_PWM = 255;

// --- Minimum Effective PWM (Motor Deadband Compensation) ---
const int MIN_EFFECTIVE_PWM = 60; // <--- ADJUST THIS VALUE!

// --- Global Motor State Variables ---
int currentMotorPWM = MIN_EFFECTIVE_PWM; // Default starting PWM
int motorDirection = 0; // 0=Stop, 1=Forward, -1=Backward

// --- ESP-01 Wi-Fi Settings ---
#define ESP_SERIAL Serial1 // Using Serial1 on Mega (Pins 18 TX1, 19 RX1)
const long ESP_BAUD_RATE = 115200;

// Your Wi-Fi credentials
const char* WIFI_SSID = "PLIJ";    // Your Hotspot Name
const char* WIFI_PASSWORD = "PLIJCONNECT"; // Your Hotspot Password

// TCP Server Settings
const int SERVER_PORT = 80; // Port for the TCP server

void setup() {
  Serial.begin(9600); // For debugging output to PC
  ESP_SERIAL.begin(ESP_BAUD_RATE); // For communication with ESP-01

  Serial.println("Robot Controller Starting (Simple Mode)...");
  Serial.println("Initializing Motors...");

  // Motor Driver Pin Setup for BOTH Motors
  pinMode(ENAR, OUTPUT);
  pinMode(IN3R, OUTPUT);
  pinMode(IN4R, OUTPUT);

  pinMode(ENAL, OUTPUT);
  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);

  // Initialize motors to stop
  stopMotors();
  delay(1000); // Give motors a moment to settle

  Serial.println("Connecting ESP-01 to Wi-Fi and starting server...");
  setupESP8266(); // Call function to configure ESP-01
  Serial.println("ESP-01 Setup Complete. Ready for commands!");
  Serial.println("----------------------------------------");
  Serial.println("Use Telnet to connect to robot's IP on port 80 (e.g., telnet 192.168.X.Y 80)");
  Serial.println("Commands: FWD, BWD, STOP, PWM [0-255]");
}

void loop() {
  if (ESP_SERIAL.available()) {
    char cmd = ESP_SERIAL.read();  // Read a single character
    Serial.print("Received: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'F':
        motorDirection = 1;
        currentMotorPWM = 150;
        setMotors(motorDirection, currentMotorPWM);
        Serial.println("Moving Forward");
        break;

      case 'S':
        motorDirection = 0;
        stopMotors();
        Serial.println("Stopped");
        break;

      // You can add more single-char commands later, e.g.:
      // case 'B':
      //   motorDirection = -1;
      //   currentMotorPWM = 150;
      //   setMotors(motorDirection, currentMotorPWM);
      //   Serial.println("Moving Backward");
      //   break;

      default:
        Serial.println("Unknown command");
        break;
    }
  }
}

// --- Motor Control Functions (Simplified) ---

void setMotors(int direction, int pwmValue) {
  int actualPWM = constrain(pwmValue, MIN_PWM, MAX_PWM);

  if (actualPWM > 0 && actualPWM < MIN_EFFECTIVE_PWM) {
    actualPWM = MIN_EFFECTIVE_PWM; // Apply deadband compensation
  } else if (actualPWM < 0 && actualPWM > -MIN_EFFECTIVE_PWM) { // For backward
    actualPWM = -MIN_EFFECTIVE_PWM;
  }


  if (direction == 1) { // Forward
    digitalWrite(IN3R, HIGH); // Right Motor Forward
    digitalWrite(IN4R, LOW);
    digitalWrite(IN1L, LOW);  // Left Motor Forward (adjust if your L298N is different)
    digitalWrite(IN2L, HIGH);
    analogWrite(ENAR, actualPWM);
    analogWrite(ENAL, actualPWM);
  } else if (direction == -1) { // Backward
    digitalWrite(IN3R, LOW);  // Right Motor Backward
    digitalWrite(IN4R, HIGH);
    digitalWrite(IN1L, HIGH); // Left Motor Backward
    digitalWrite(IN2L, LOW);
    analogWrite(ENAR, actualPWM);
    analogWrite(ENAL, actualPWM);
  } else { // Stop
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(ENAR, 0);
  analogWrite(ENAL, 0);
  digitalWrite(IN3R, LOW);
  digitalWrite(IN4R, LOW);
  digitalWrite(IN1L, LOW);
  digitalWrite(IN2L, LOW);
  Serial.println("Motors Stopped.");
}

// --- ESP-01 Communication Functions ---
void sendATCommand(const char* command, unsigned long timeout = 5000) {
  Serial.print("Sending: ");
  Serial.println(command);
  ESP_SERIAL.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (ESP_SERIAL.available()) {
      char c = ESP_SERIAL.read();
      Serial.write(c); // Echo ESP's response to PC serial monitor
    }
  }
}

void sendESPResponse(int connectionId, String message) {
  String cmd = "AT+CIPSEND=" + String(connectionId) + "," + String(message.length());
  ESP_SERIAL.println(cmd);
  Serial.print("Sending to client "); Serial.print(connectionId); Serial.print(": "); Serial.println(message);
  delay(10); // Short delay for ESP to process AT+CIPSEND command

  unsigned long startTime = millis();
  while (millis() - startTime < 1000) { // Max 1 second wait for '>'
    if (ESP_SERIAL.available()) {
      char c = ESP_SERIAL.read();
      Serial.write(c);
      if (c == '>') {
        ESP_SERIAL.println(message);
        ESP_SERIAL.println(); // Send a newline after the message for Telnet
        break;
      }
    }
  }
}

void setupESP8266() {
  // Reset ESP-01
  sendATCommand("AT+RST", 2000);
  delay(2000);

  sendATCommand("AT"); // Check AT command
  sendATCommand("AT+CWMODE=1"); // Set Wi-Fi Mode to Station (Client)

  // Connect to Wi-Fi
  String connectCommand = "AT+CWJAP=\"";
  connectCommand += WIFI_SSID;
  connectCommand += "\",\"";
  connectCommand += WIFI_PASSWORD;
  connectCommand += "\"";
  sendATCommand(connectCommand.c_str(), 10000);

  sendATCommand("AT+CIPMUX=1"); // Enable multiple connections
  String serverCommand = "AT+CIPSERVER=1," + String(SERVER_PORT); // Start TCP Server
  sendATCommand(serverCommand.c_str());

  sendATCommand("AT+CIFSR"); // Get and display IP address
}