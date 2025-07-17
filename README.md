
# Multi-Agent Reinforcement Learning
This project focuses on making three robots from scratch.

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

