#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Variables for angle calculation
float angleZ = 0.0; // Our calculated angle around the Z-axis (relative rotation)
unsigned long previousMicros; // To store the timestamp of the previous loop iteration

// Gyroscope bias variable
float gyroZ_bias = 0.0; // This will store the average offset from calibration

void setup(void) {
  Serial.begin(115300); // Changed to 115300 for safety, some systems prefer this over 115200

  // Try to initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip!");
    Serial.println("Check wiring, I2C address, and power.");
    while (1) {
      delay(10); // Halt program if MPU6050 is not found
    }
  }
  Serial.println("MPU6050 Found and Initialized!");

  // Set sensor ranges (good for general movement)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Max 500 deg/s is about 8.7 rad/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Filters out high-frequency noise

  delay(100); // Give the sensor a moment to stabilize

  // --- GYROSCOPE CALIBRATION ---
  Serial.println("Calibrating Gyroscope... KEEP ROBOT STILL!");
  long calibrationSum = 0;
  int calibrationSamples = 200; // Take many samples for a good average

  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t g, a, temp; // Gyroscope event
    mpu.getEvent(&a, &g, &temp); // Read data
    calibrationSum += g.gyro.z; // Add to sum (convert to deg/s for consistency)
    delay(10); // Small delay between readings
  }
  gyroZ_bias = (float)calibrationSum / calibrationSamples;
  Serial.print("GyroZ Bias: ");
  Serial.print(gyroZ_bias);
  Serial.println(" degrees/second");
  Serial.println("Calibration Complete. You can move the robot now.");

  // --- Serial Plotter Legend ---
  Serial.println("AngleZ (degrees)"); // This line will show in Serial Monitor for clarity
  
  // Initialize previousMicros with the current time after calibration
  previousMicros = micros();
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- Calculate Delta Time (dt) ---
  unsigned long currentMicros = micros(); // Get current time in microseconds
  float dt = (currentMicros - previousMicros) / 1000000.0; // Convert delta time to seconds
  previousMicros = currentMicros; // Update previous time for the next iteration

  // --- Bias Correction ---
  // Subtract the measured bias from the current gyro Z reading
  float corrected_gyro_z_rad_per_sec = g.gyro.z - (gyroZ_bias); // Convert bias back to rad/s for subtraction
  // Now convert to degrees/second for integration:
  float corrected_gyro_z_deg_per_sec = corrected_gyro_z_rad_per_sec * (180.0 / PI);


  // --- Integrate Corrected Gyroscope Z-axis for Angle ---
  // Add the calculated angle change to our total angleZ
  angleZ += corrected_gyro_z_deg_per_sec * dt;

  // --- Optional: Keep angle within 0-360 degrees if desired ---
  // Useful if you only care about the current rotation within a single circle.
  // if (angleZ >= 360.0) angleZ -= 360.0;
  // if (angleZ < 0.0) angleZ += 360.0;

  // --- Output for Serial Plotter ---
  // Serial.print(g.gyro.z);
  // Serial.print(",");
  // Serial.println(angleZ); // Plot only the calculated Z-angle

  // Small delay to control data rate for plotting. Adjust as needed.
  // Too fast can overwhelm the plotter, too slow won't show smooth changes.
  delay(100); // Slightly increased delay for better stability in Plotter
}