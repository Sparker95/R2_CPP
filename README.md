# R2 - LIDAR Indoors Robot Car

<img height="310" alt="Image" src="https://github.com/user-attachments/assets/78f9a7c9-714d-4541-a9c6-6d71fe3531a7" /><img height="310" alt="Image" src="https://github.com/user-attachments/assets/e2cefe24-f1e6-436e-bfc1-c9d0f22b7917" />

R2 is a spare time project aimed at practicing of indoors robot navigation: SLAM (Simultaneous Localization and Mapping), path finding, driving, map exploration, and everything related.

R2 name is short of "Robot 2", since it is my second robot build :)

Ackermann platform is not practical indoors, and was chosen for fun & curiosity reasons.

**Key components** are:
- [LRP S10 Blast TC3 RC car chassis](https://www.lrp.cc/en/product/120203/)
- [Slamtec A1M8 LIDAR](https://www.slamtec.com/en/lidar/a1)
- [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) runs all processing of LIDAR and chassis data, builds map, etc.
- [Arduino Micro](https://store.arduino.cc/products/arduino-micro) main job is PID controller for stable motor speed.
- Qt-based control panel on PC toggles modes and visualizes data.

## Current functionality
**WIP** Video of MainWindow, video of actual car driving
**WIP** list of features: FastSLAM, path finding, path following, replanning, exploration of map

## Full list of hardware components
**WIP**
LIDAR
Chassis, encoder mod
Servo, mod
ESC
Arduino micro
Raspberry Pi
Power supply
Battery

## Repo structure
**WIP** This repository contains C++ code, the other contains Python code WIP
List programs: Common folder, Config folder, r2pi, MotorControl, RClient
Used libs: Eugen, Tracy, Pigpio

## Problems solved in software
**WIP** SLAM, path finding, path following, exploration

## References and thanks
the SLAM course, 


