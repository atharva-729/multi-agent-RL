
# Multi-Agent Reinforcement Learning
This project focused on designing and assembling three ground robots to study how multiple agents can learn to work together in a shared environment. Each robot was built from scratch using Arduino and included encoders, IMUs, and ultrasonic sensors for motion tracking and obstacle detection. I developed a control system that handled movement, communication, and decision-making in real time. To make the robots more reliable, I also built a Bayesian framework to model how different failures could occur and affect learning.

---

## Files in this Repository

### Folder: `Arduino files`

* Contains the Arduino sketches to be uploaded to the robots.
* Each file is labeled for the respective robot.
* Robot 3 has **two versions**: open loop control and closed loop control (not sure which one works best).
* All files include comments and documentation.

### Folder: `Development files`

* Contains code and scripts used during development and hardware testing.
* **Mainly for debugging purposes.**
* You do **not** need any file from this folder to run the robots or the trained model.

### File: `list of failure causes.md`

* Lists the failure modes, effects, and causes.
* Also includes the probabilities used to train the reinforcement learning model.

### File: `precautions.md`

* Lists important precautions related to the robot and hardware.
* Covers all major components used in the robots.

### File: `coordinate_control.py`

* This is the Python script used to send coordinates to the robot.
* The same script is used for all three robots — **only the COM port needs to be updated**.

### File: `what to do.md`

* Step-by-step guide for connecting the Bluetooth modules.
* Also explains which code to run and in what order.

---

### Arduino Pinout

| **Component**                    | **Signal/Pin**           | **Connected to Arduino Mega Pin** | **Notes**                                |
| -------------------------------- | ------------------------ | --------------------------------- | ---------------------------------------- |
| **L298 Motor Driver**            | ENA                      | 9                                 | Left motor PWM control                   |
|                                  | IN1                      | 8                                 | Left motor direction                     |
|                                  | IN2                      | 7                                 | Left motor direction                     |
|                                  | ENB                      | 10                                | Right motor PWM control                  |
|                                  | IN3                      | 5                                 | Right motor direction                    |
|                                  | IN4                      | 4                                 | Right motor direction                    |
| **Encoders**                     | Left Encoder Output      | 3                                 | Interrupt-capable pin                    |
|                                  | Right Encoder Output     | 2                                 | Interrupt-capable pin                    |
| **Bluetooth (HC-05)**            | RX (via voltage divider) | 18 (TX1)                          | Mega TX1 to HC-05 RX via voltage divider |
|                                  | TX                       | 19 (RX1)                          | HC-05 TX to Mega RX1                     |
| **MPU6050**                      | SDA                      | 20                                | I2C data                                 |
|                                  | SCL                      | 21                                | I2C clock                                |
| **Ultrasonic Sensors (HC-SR04)** | TRIG\_LEFT               | 43                                | Left sensor trigger                      |
|                                  | ECHO\_LEFT               | 42                                | Left sensor echo                         |
|                                  | TRIG\_RIGHT              | 33                                | Right sensor trigger                     |
|                                  | ECHO\_RIGHT              | 32                                | Right sensor echo                        |
| **Power**                        | VIN                      | Breadboard left power rail        | External power input to Arduino          |
|                                  | GND                      | Ground rail                       | Common ground                            |
|                                  | 5V                       | Breadboard right power rail       | Powers all sensors (except 1 HC-SR04)    |

