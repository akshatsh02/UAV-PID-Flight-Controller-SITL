# UAV-PID-Flight-Controller-SITL
ESP32 and MPU-6050 based closed-loop PID flight stabilization firmware (Wokwi SITL).


An embedded avionics payload simulating active aerodynamic stabilization and real-time telemetry streaming, developed following the BSERC Def-Space internship.

## Hardware Architecture (Simulated)
* **Flight Controller:** ESP32 (Dual-core, 50Hz control loop)
* **IMU:** MPU-6050 (6-axis spatial data via I2C)
* **Actuators:** PWM-driven Micro-Servos (Elevons)

## Control Logic & Sensor Fusion
* **Complementary Filter:** Fuses 98% Gyroscope and 2% Accelerometer data to eliminate high-frequency motor noise and sensor drift.
* **PID Controller:** Closed-loop aerodynamic correction replacing linear actuation.
  * $K_p = 1.5$ (Immediate counter-rotational force)
  * $K_i = 0.05$ (Steady-state error accumulation against sustained crosswinds)
  * $K_d = 0.8$ (Momentum braking to prevent pendulum overshoot)

## Live Simulation
[Click here to run the SITL simulation in your browser](https://wokwi.com/projects/476946542890987521)

## Telemetry & Demonstration

<img width="870" height="672" alt="image" src="https://github.com/user-attachments/assets/d1e7e5cf-d645-4ec4-818c-e64beea34a15" />


https://github.com/user-attachments/assets/509221ea-88f6-4aea-ae8f-2ea3851c819f




