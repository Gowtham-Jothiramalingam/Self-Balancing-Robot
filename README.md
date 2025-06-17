# Self-Balancing Robot Firmware

## Overview
This Arduino sketch implements the core control logic for a self-balancing robot using an MPU-6050 IMU sensor. It reads accelerometer and gyroscope data via I2C, calculates the robot's tilt angle using a complementary filter, and applies a PID controller to drive motors to maintain balance.

---

## Hardware Requirements
- Microcontroller board compatible with Arduino IDE (e.g., ESP8266, Arduino Uno with pin remapping)
- MPU-6050 IMU sensor (I2C address `0x68`)
- Two DC motors with H-bridge motor driver
- Motor driver connected to pins:
  - Right motor: `rmotor1 = D3`, `rmotor2 = D4`
  - Left motor: `lmotor1 = D5`, `lmotor2 = D6`
- Power supply suitable for motors and microcontroller

---

## Pin Configuration
| Function            | Pin  |
|---------------------|-------|
| Right Motor Forward  | D3    |
| Right Motor Backward | D4    |
| Left Motor Forward   | D5    |
| Left Motor Backward  | D6    |
| I2C SDA             | 4     |
| I2C SCL             | 5     |

> *Note:* Pins `D3` to `D6` correspond to digital pins on the microcontroller. Adjust accordingly based on your hardware.

---

## Features
- **IMU Data Acquisition:** Reads raw accelerometer and gyroscope data from MPU-6050.
- **Angle Calculation:** Converts raw sensor data into pitch and roll angles using trigonometric functions.
- **Complementary Filter:** Combines accelerometer and gyro data for stable angle estimation.
- **PID Control:** Uses proportional and derivative terms to calculate motor PWM signals to correct the robot's tilt.
- **Motor Control:** Drives motors forward or backward based on PID output to maintain balance.
- **Safety Halt:** Stops motors if tilt angle exceeds ±45°, preventing damage.

---

## How It Works

1. **Initialization:**
   - Starts I2C communication with MPU-6050.
   - Configures motor control pins as outputs.
   - Initializes serial communication for debugging.

2. **Sensor Reading:**
   - Reads accelerometer data (X, Y, Z axes).
   - Reads gyroscope data (X, Y axes).

3. **Angle Computation:**
   - Calculates pitch and roll from accelerometer data.
   - Calculates angular velocity from gyroscope data.
   - Applies complementary filter to fuse accelerometer and gyro angles.

4. **PID Control:**
   - Calculates error between current pitch angle and desired angle (0°).
   - Computes proportional and derivative parts of PID.
   - Calculates PID output to determine motor speed and direction.

5. **Motor Driving:**
   - If tilt is negative, motors rotate anti-clockwise.
   - If tilt is positive, motors rotate clockwise.
   - If tilt exceeds ±45°, motors halt immediately.

6. **Loop Timing:**
   - Uses elapsed time between loops for accurate angle integration and PID derivative calculation.
   - Avoids delays and serial prints in main loop for fast response (debug prints are commented out).

---
## PID Tuning Guide

Tuning the PID controller is crucial for stable balancing.

### 1. Understand PID Components
- **Proportional (P):** Corrects error proportionally to the difference between desired and current angle. Too high causes oscillations; too low causes sluggish response.
- **Integral (I):** Corrects accumulated past errors. Helps eliminate steady-state error but can cause overshoot if too high.
- **Derivative (D):** Reacts to the rate of error change. Helps dampen oscillations and smooth response.

### 2. Start With P Only
- Set `ki = 0` and `kd = 0`.
- Gradually increase `kp` from 0 upwards until the robot starts to respond and balance but may oscillate.
- Example: Start with `kp = 10`, increase in steps of 5.

### 3. Add D to Reduce Oscillations
- Increase `kd` slowly to dampen oscillations.
- Typical starting value: `kd = 0.5` to `1.0`.
- Adjust until oscillations reduce but response remains quick.

### 4. Add I if Needed
- If the robot drifts over time or doesn’t hold position, add a small integral term.
- Start with a very small `ki` like 0.01.
- Be cautious: too much integral causes instability.

### 5. Fine-Tune
- Iterate adjusting `kp`, `ki`, and `kd` for smooth, stable balancing.
- Observe behavior carefully after each change.
- Use serial prints to monitor PID output and angles if needed.

### 6. Tips
- Avoid too high PID values to prevent motor saturation.
- Ensure your loop timing (`elapsedTime`) is accurate for derivative calculation.
- Tune on a stable surface with minimal disturbances.
---
## Usage Instructions

1. **Wiring:**
   - Connect MPU-6050 to I2C pins (SDA to pin 4, SCL to pin 5).
   - Connect motor driver inputs to pins D3, D4, D5, and D6.
   - Supply appropriate power to motors and microcontroller.

2. **Upload Code:**
   - Open the sketch in Arduino IDE.
   - Select the correct board and port.
   - Upload the code.

3. **Tuning PID:**
   - Adjust `kp`, `ki`, and `kd` constants in the code to optimize balancing performance.
   - Start with `kp=25`, `ki=0`, `kd=0.8` as provided.
   - Increase or decrease values based on robot response.

4. **Testing:**
   - Power on the robot on a flat surface.
   - Observe motor behavior and adjust PID parameters if necessary.
   - Use serial prints for debugging by uncommenting lines in the loop.

---

## Notes
- **Common Ground:** Ensure all grounds (microcontroller, motor driver, battery) are connected together.
- **No Delays in Loop:** Delays are avoided to maintain fast control loop timing.
- **Serial Prints:** Should be disabled in production to avoid slowing down the loop.
- **Safety:** The robot halts motors if the tilt angle exceeds ±45° to prevent falls.
- **Power Supply:** Ensure motors have sufficient current supply to respond effectively.

---

## Code Structure 

| Function       | Description                              |
|----------------|------------------------------------------|
| `setup()`      | Initializes I2C, motors, and serial      |
| `loop()`       | Main control loop: reads sensors, computes PID, drives motors |
| `clockw()`     | Drives motors clockwise                   |
| `anti()`       | Drives motors anti-clockwise              |
| `halt()`       | Stops all motors                          |

---

## License
This code is provided as-is for educational and hobbyist use. Modify and adapt freely.
