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

## List of hardware components and modifications

**LIDAR**

[A1M8 LIDAR by Slamtec](https://www.slamtec.com/en/lidar/a1) is the core instrument for navigation. This is cheapest LIDAR I found nearby, but is adequate for experiments. It operates based on triangulation principles, and not time-of-flight. It outputs data via UART, and is connected straight to Raspberry Pi.

**Chassis & modifications**

An RC car chassis is used as the robot base. [S10 Blast TC3 chassis by LRP](https://www.lrp.cc/en/product/120203/) is a 4WD car, with ~25cm wheel base, and is meant for custom RC car builds.

- 🔧 A custom incremental single-channel magnetic encoder was installed. It connects to the Arduino and has dual purpose: speed sensor for motor PID controller; measurement of distance traveled between updates of navigation algorithm. The encoder consists of a 3D-printed disc holding 20 magnets, and a 3D-printed holder for TLE4935L Hall sensor.

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
- [QuickRun WP 1060 brushed electronic speed controller (ESC)](https://www.hobbywing.com/en/products/quicrun-wp-1060-brushed55) was used as motor driver. It receives PWM signal as input (usual for RC models), and can drive the motor forward and backward at different speed. Also it can perform braking by shorting the motor.
- 3000 mAh 7.2V NiMH battery

**Electronic parts**

- [Arduino Micro](https://store.arduino.cc/products/arduino-micro)
- Connectors, prototype board, etc
- [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) with plastic case.
- XL6009 buck-boost converter. Or any converter outputting 5V ~2A from the battery.
- Voltmeter module with LED display, to know battery voltage.

## Principles of operation

Solution of the SLAM problem is central one, and many other decisions revolve around it. SLAM is solved by FastSLAM algorithm, which utilizes a particle filter. Each rotation of the LIDAR produces a new scan and constitutes one **update cycle**. Also we must know difference of robot's pose (x,y, dir) between the two updates. The offset is calculated based on basic mathematical model of car motion, which relies on data from extra sensors. The car motion model also helps compensate the LIDAR scan distortion caused by movement.

The extra chassis sensor data includes:
- **Steering angle**, calculated from voltage of servo potentiometer.
- **Number of pulses from incremental encoder** mounted on the differential input shaft.

Providing this data, and more, is handled by the **Arduino Micro** board:
- Generation of PWM signals for the servo and ESC (electronic speed controller). [50 Hz PWM signal](https://en.wikipedia.org/wiki/Servo_control) is traditionally used in hardware for RC models.
- Counting encoder pulses.
- PID control of motor speed.
- Measurement of servo position by ADC.
- Communication with RPi via UART.

Thus we have a basic chassis, executing steering and movement commands, and providing extra sensor data. FastSLAM, running on Raspberry Pi, solves the mapping problem and produces the map when car is moving, regardless why it is moving (exploring the map itself, following path, or simply remotely controlled).

Extra autonomous capabilities are achieved "on top" of FastSLAM output. Right now, the solutions chosen here are _naive implementations and not proper solutions_ how to handle results from a particle filter:
- The map of the most likely particle is assumed the true map.
- The mean pose of all particles is assumed the true pose.
- 
This obviously _can not deal with multimodal distribution of particles_ (resulting from unresolved loops in environment). However, this turned out sufficient for the basic test environment.

Sections below briefly describe operation of each functionality:

### Path following
Path following utilizes [pure pursuit algorithm](https://publications.ri.cmu.edu/storage/publications/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf).

### Path finding
Path finding utilizes A*. Each next node is one of 6 possible transitions: forward, fwd+left, fwd+right, backward, bwd+left, bwd+right. Each transition the car moves by a discrete distance straight, or also rotates by discrete amount of degrees. Car collision is tested by rectangle rasterization against the map.

A few tricks keep amount of explored nodes sane:
- "Skeleton" of the map is constructed based on [distance transform](https://en.wikipedia.org/wiki/Distance_transform). Exploration along the map skeleton (within fixed distance) has smaller cost in A*.
- Grid-hashmap detects already visited locations, such as car completing a full circle across several iterations.

Images: _Original map and map skeleton_

<img height="150" alt="Image" src="https://github.com/user-attachments/assets/20f14123-5e68-4f00-8645-edda7b3d50b7" />
<img height="150" alt="Image" src="https://github.com/user-attachments/assets/29b632ae-4497-49ac-8464-7640580ad243" />

Images: _pathfinding starting from tight spot; all explored positions are visualized as red dots; final path is blue line._

<img height="250" alt="Image" src="https://github.com/user-attachments/assets/46dc553a-19b0-4871-95cb-e7ca8a84d22f" />
<img height="250" alt="Image" src="https://github.com/user-attachments/assets/3fbb5ef1-2e61-4334-b406-b96f00ecc31c" />
<img height="250" alt="Image" src="https://github.com/user-attachments/assets/ba01c045-923a-4d6f-bd32-7f1e5e913554" />

It's worth noting that the paths which follow map skeleton can be not nicest visually and not shortest. But they maximize safety of car, since it will drive furthest possible from map obstacles.

### Replanning
The currently-chosen path is actively monitored for obstructions along the way. If obstruction is detected, car stops and alternative path is calculated. Map is often 'noisy', as same cells change their value, and often there is too much replanning as seen on video.

### Exploration
Exploration means visiting all unexplored map areas and building whole map.

Exploration is implemented as a basic greedy algorithm. At each iteration it selects most 'interesting' place and drives there, then repeats.

First the exploration algorithm groups unexplored 'frontier' cells, then scores those groups. Bigger and nearest groups have highest score.

Images: _Original map at start of exploration; discovered frontier areas and their scores for exploration._

<img height="250" alt="Image" src="https://github.com/user-attachments/assets/6c8ac0f8-4559-4788-8b59-66eee8dc6cbb" /><img height="250" alt="Image" src="https://github.com/user-attachments/assets/d561807e-da5f-4847-9ef0-01b0aaea97c6" />

## Control Panel
Finally, there is the Control Panel program, running in Windows. It and connects to the Raspberry Pi software through TCP. It can receive full data from sensors, save it to files, or send back basic commands.

## Repo structure
This repository contains C++ code. [The other repository](https://github.com/Sparker95/R2_Python) contains Python code.

**Common** folder contains most of C++ code which is independent on platform. The FastSLAM, path finding, etc.

**r2pi** is the project for Raspberry Pi. It also contains lots of _platform-dependent_ code, such as the serial interfaces with LIDAR/Arduino, etc.

**MotorControl** is the code for Arudiono Micro.

**RClient** is the control panel QT project. It's called a client because it connects to **r2pi** as TCP client.

**r2offline_pi** and **r2offline_win** are projects for testing purposes, to run on Linux or Windows. They can be used for reading sensor data in offline mode, or other tests.

## Used libraries
[Eigen C++ library for linear algebra](https://libeigen.gitlab.io/)
[Tracy Profiler](https://github.com/wolfpld/tracy) greatly helps profile performance on Raspberry Pi.
[Pigpio library](https://abyz.me.uk/rpi/pigpio/) is used for PWM control of LIDAR motor.

## References and thanks
the SLAM course, 

