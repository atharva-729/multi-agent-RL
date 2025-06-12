#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200); // Set your Serial Monitor and Plotter to this baud rate

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    // Loop forever if MPU6050 is not found
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found and Initialized!");

  // Set accelerometer range to +-8G (Good for general movement)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Set gyro range to +- 500 deg/s (Good for general rotation)
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Set filter bandwidth to 21 Hz (Filters out high-frequency noise)
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100); // Give the sensor a moment to stabilize

  // --- LEGEND FOR SERIAL MONITOR & PLOTTER ---
  // This line will print once to the Serial Monitor to tell you what each "Value" in the plotter is.
  Serial.println("AccX,AccY,AccZ,GyroX,GyroY,GyroZ,Temp");
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Output for Serial Plotter ---
  // The plotter expects comma-separated numerical values on a single line.
  // It won't display the names in the plotter itself, but the legend above helps!

  Serial.print(a.acceleration.x); // Value 0: Acceleration X
  Serial.print(",");
  Serial.print(a.acceleration.y); // Value 1: Acceleration Y
  Serial.print(",");
  Serial.print(a.acceleration.z); // Value 2: Acceleration Z
  Serial.print(",");
  Serial.print(g.gyro.x); // Value 3: Rotation X
  Serial.print(",");
  Serial.print(g.gyro.y); // Value 4: Rotation Y
  Serial.print(",");
  Serial.print(g.gyro.z); // Value 5: Rotation Z
  Serial.print(",");
  Serial.println(temp.temperature); // Value 6: Temperature (use println for the last value to create a new line)

  // You can adjust this delay. Too fast might overwhelm the plotter, too slow
  // might not show smooth changes. 50ms is usually a good starting point.
  delay(100);
}