### How to Run the Robot

1. **Upload the code** to the correct Arduino board.
2. **Connect the Arduino to your laptop** using USB. Then **pair the Bluetooth module** (default password: `1234`).
3. **Check the Bluetooth COM port**:

   * Go to **Settings > Bluetooth > Devices**
   * Click **"More devices and printer settings"**
   * Find your Bluetooth module, **double-click it**
   * Go to the **Services** tab – the **COM port number** will be listed (e.g., COM5).
4. **Update the port number** in the Python script with the correct COM port.
5. In the Python script, **enter the coordinates** (in millimetres) that the robot should move to.
6. **Unplug the Arduino from the laptop**, and **turn on the power switch** on the robot.
7. **Run the Python script**.
   If Bluetooth doesn’t connect, **run the script again** (it may take up to 5 tries).
8. The robot should start moving to the coordinates you entered.
