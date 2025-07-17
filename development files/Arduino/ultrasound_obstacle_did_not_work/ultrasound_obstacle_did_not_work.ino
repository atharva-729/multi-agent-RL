#define TRIG_LEFT 43
#define ECHO_LEFT 42
#define TRIG_RIGHT 33
#define ECHO_RIGHT 32

#define OBSTACLE_THRESHOLD_CM 30
#define MIN_VALID_DISTANCE 5
#define INVALID_READING -1

unsigned long lastSensorReadTime = 0;
const unsigned long SENSOR_INTERVAL = 500; // ms

int leftObstacleCounter = 0;
int rightObstacleCounter = 0;

long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 40000); // 40 ms timeout
  long distance = duration * 0.034 / 2.0;

  if (distance < MIN_VALID_DISTANCE || duration == 0) {
    return INVALID_READING;
  }

  return distance;
}

void setup() {
  Serial.begin(9600); // For debug
  // Serial1.begin(9600); // Uncomment if using HC-05 on Serial1

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}

void loop() {
  unsigned long now = millis();

  if (now - lastSensorReadTime >= SENSOR_INTERVAL) {
    lastSensorReadTime = now;

    long distL = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
    delay(20); // wait before pinging next sensor
    long distR = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("left distance: ");
    Serial.print(distL);
    Serial.print(" | right distance: ");
    Serial.println(distR);

    // Check left
    if (distL != INVALID_READING && distL < OBSTACLE_THRESHOLD_CM) {
      leftObstacleCounter++;
    } else {
      leftObstacleCounter = 0;
    }

    // Check right
    if (distR != INVALID_READING && distR < OBSTACLE_THRESHOLD_CM) {
      rightObstacleCounter++;
    } else {
      rightObstacleCounter = 0;
    }

    // Debounced obstacle detection
    if (leftObstacleCounter >= 2) {
      Serial.println("LEFT OBSTACLE DETECTED");
      // Serial1.println("LEFT OBSTACLE DETECTED"); // if sending via Bluetooth
    }

    if (rightObstacleCounter >= 2) {
      Serial.println("RIGHT OBSTACLE DETECTED");
      // Serial1.println("RIGHT OBSTACLE DETECTED"); // if sending via Bluetooth
    }
  }
}
