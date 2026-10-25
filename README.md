# R2 - LIDAR Indoors Robot Car

<img height="250" alt="Image" src="https://github.com/user-attachments/assets/78f9a7c9-714d-4541-a9c6-6d71fe3531a7" /><img height="250" alt="Image" src="https://github.com/user-attachments/assets/e2cefe24-f1e6-436e-bfc1-c9d0f22b7917" />

R2 is a spare time project aimed at practicing of indoors robot navigation: SLAM (Simultaneous Localization and Mapping), path finding, driving, map exploration, and everything related.

R2 name is short of "Robot 2", since it is my second robot build :)

Ackermann platform is not practical indoors, and was chosen for fun & curiosity reasons.

**Key components** are:
- [LRP S10 Blast TC3 RC car chassis](https://www.lrp.cc/en/product/120203/)
- [Slamtec A1M8 LIDAR](https://www.slamtec.com/en/lidar/a1)
- [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) runs all processing of LIDAR and chassis data, builds map, etc.
- [Arduino Micro](https://store.arduino.cc/products/arduino-micro) runs PID controller for stable motor speed, interfaces the RPi with steering servo.
- Qt-based control panel on PC toggles modes and visualizes data.

## Current functionality
Video demonstrations:

[▶ YouTube: View of the car as it explores environment](https://youtu.be/uJNpeLaECnc?si=d3IO5W9ucZdIrcSA)

[▶ YouTube: View of the control panel, robot is remotely controlled](https://youtu.be/VyZWErm8OQ4?si=NTOK5pJTbdTtG3kR)

Here's a short summary of current abilities. The various aspects of implementations and their downsides are explained later.
- Remote control
- Simultaneous Localization and Mapping, powered by FastSLAM algorithm (particle filter)
- Path finding
- Path following
- Path replanning if obstructed
- Automatic exploration of the map

## Full list of hardware components

**LIDAR**

[A1M8 LIDAR by Slamtec](https://www.slamtec.com/en/lidar/a1) is the core instrument for navigation. This is cheapest LIDAR I found nearby, but is adequate for experiments. It operates based on triangulation principles, and not time-of-flight. It outputs data via UART, and is connected straight to Raspberry Pi.

**Chassis & modifications**

An RC car chassis is used as the robot base. [S10 Blast TC3 chassis by LRP](https://www.lrp.cc/en/product/120203/) is a 4WD car, with ~25cm wheel base, and is meant for custom RC car builds.

- 🔧 A custom magnetic encoder was installed. It connects to the Arduino and has dual purpose: speed sensor for motor PID controller; measurement of distance traveled between updates of navigation algorithm. The encoder consists of a 3D-printed disc holding 20 magnets, and a 3D-printed holder for TLE4935L Hall sensor.

<img height="150" alt="Image" src="https://github.com/user-attachments/assets/ec817778-cb4f-42ed-979d-330c8e8c9a6a" /><img height="150" alt="Image" src="https://github.com/user-attachments/assets/671d57c9-a447-4395-9415-34b387356070" /><img height="150" alt="Image" src="https://github.com/user-attachments/assets/4e426c57-6d98-43d5-b03b-a2dc9c05bfaa" /><img height="150" alt="Image" src="https://github.com/user-attachments/assets/ea1577ad-5745-4898-b891-9b0115869ba6" />

- 🔧 The axle linking rear and front differentials was removed, to reclaim some space inside the car. This makes the car rear-wheel-drive.
- 🔧 Custom 3D-printed rear bumper mount. Front and rear bumpers cut out of foam.
- 🔧 Last, an aluminum sheet is mounted on 3D-printed supports. Most of electronics is mounted to the sheet.

**Servo & modifications**

A Traxxas 2075X servo was used for steering, but most likely any other is suitable.

- 🔧 The servo was modified to output instant rotation position. This is done by soldering wires straight to the potentiometer.

<img width="150" alt="Image" src="https://github.com/user-attachments/assets/603391f6-8230-415e-9552-600b39b21091" />

**Other chassis parts**

- [540-size 55-turn brushed motor by YeahRacing](https://www.yeahracing.com/yeah-racing-hackmoto-v2-55t-540-brushed-motor-mt-0016-00056719)
- [QuickRun WP 1060 brushed electronic speed controller (ESC)](https://www.hobbywing.com/en/products/quicrun-wp-1060-brushed55) was used as motor driver. It receives PWM signal as input (usual for RC models), and can drive the motor forward and backward at different speed.
- 3000 mAh NiMH battery

**Electronic parts**

- [Arduino Micro](https://store.arduino.cc/products/arduino-micro)
- Connectors, prototype board, etc
- [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) with plastic case, Micro SD card.
- XL6009-based buck-boost converter, or anything able to provide 5V and a couple of Amps for Raspberry Pi and LIDAR.
- Voltmeter module with LED display, to know battery voltage.

## Problems solved in software
**WIP** SLAM, path finding, path following, exploration

## Repo structure
**WIP** This repository contains C++ code, the other contains Python code WIP
List programs: Common folder, Config folder, r2pi, MotorControl, RClient
Used libs: Eugen, Tracy, Pigpio

## References and thanks
the SLAM course, 


