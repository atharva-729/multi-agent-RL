
## Precautions to Take

### 1. Batteries

* Check voltage level with a multimeter (dial at 20V DC). If it goes below 3.5V, charge them.
* When not in use, take out the middle battery from each robot. Store it in a plastic bag preferably.
* **These must not short-circuit. Never let both terminals touch the same metal piece/wire.**
* Let them charge till 4.25V and then take them out (check with multimeter).
* The difference in voltage of the three batteries on a single robot should not be greater than 0.05V.
* You may label batteries for different robots to avoid confusion.
* These batteries do not drain easily, but do check the battery level every 2 weeks.

### 2. DC-DC Converter

* You must ensure that the soldering remains intact on all four terminals of the converter. All four corners have a terminal.
* The LED will indicate if there is power available at the output terminals (ones on the side of the switch). The LED on the converter of the first robot is damaged; you must check with a multimeter.
* Do check the output voltage level on the output terminals of the converter. For the first and second robot, it should be **9V ± 0.1V**, and for the third robot, it should be **7.5V ± 0.1V**.

### 3. Switch

* O side pushed in: **Off**
* Turn on the switch only after you have made sure that all connections are secured, especially the ones on the left power rail of the breadboard.

### 4. Breadboard

* The left-side power rails carry **9V** (or **7.5V** for the third robot). Only the motor driver and the Arduino have live (red) wires connected to this rail. Always check these connections thoroughly before turning the switch on.
* There must be a wire connecting the grounds on both sides of the breadboard.
* The right-side power rail carries **5V** for all the sensors and modules.

### 5. Motor Driver

* When you turn on the switch, the red LED on the motor driver must turn on. If it does not, it might have stopped working.

### 6. Arduino

* Make sure the wires carrying **9V** (or **7.5V**) to the Arduino are tightly connected and in their correct pin slots. Any mismatch can and will fry the Arduino.
* Refer to the pinout diagram of the Arduino that I provide and just once ensure all wires are in their respective pin slots.
* If using the robots after some time, push all wires into their slots once. They might have come loose.
* When checking or installing the sensors and modules, power the Arduino from your laptop, not from the batteries.
* **Do not power the Arduino from both the batteries and the laptop.**

### 7. MPU6050 (Gyroscope + Accelerometer)

* Just make sure the wires are fully connected.

### 8. Ultrasonic Sensors (HC-SR04)

* They must point straight, parallel to the ground or tilted very slightly towards the ground.

### 9. Bluetooth Module (HC-05)

* The pins on the module must be all the way in the breadboard.
* Continuous blinking on the module means it is not connected to any device.

### 10. Hall Effect Sensors (Encoders)

* While mounting, make sure you get all eight pulses in one rotation of the wheel.

### 11. Magnets

* These might come loose and get stuck to one another. In that case, carefully stick the magnet back to its original place, but the polarity must remain the same. To check polarity, just check with the other already mounted magnets and stick them back.
* Press the magnets in whenever you start working with the robots. The double tape adhesive loses its grip after some time.

### 12. Wires

* Do check every single connection is well enough every time you start.

