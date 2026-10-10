# R2 - LIDAR Home Robot on Ackermann Chassis
R2 is a spare time project aimed at practicing of indoors robot navigation: SLAM (Simultaneous Localization and Mapping), path finding, path following, map exploration, and everything related.

R2 name is short of "Robot 2", since it is my second robot build :)

Ackermann platform is not practical indoors, and was chosen for fun & curiosity reasons.

**Key components** are [LRP S10 Blast TC3 RC car chassis](https://www.lrp.cc/en/product/120203/), [Slamtec A1M8 LIDAR](https://www.slamtec.com/en/lidar/a1), [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/), [Arduino Micro](https://store.arduino.cc/products/arduino-micro).

**Architecture** is:
- The Raspberry Pi does all processing, including LIDAR data.
- Arduino Micro main job is PID controller for stable motor speed.
- Qt-based control panel on PC sends controls and visualizes data.

<img width="700" alt="Image" src="https://github.com/user-attachments/assets/78f9a7c9-714d-4541-a9c6-6d71fe3531a7" />
