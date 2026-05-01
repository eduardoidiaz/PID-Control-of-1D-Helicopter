# RP2040 PID Helicopter Control System

Real-time closed-loop control of a 1-D helicopter platform using the RP2040 microcontroller, an MPU6050 IMU, PWM motor control, VGA graphics, and a configurable PID controller.

## Overview

This project implements a real-time PID control system for stabilizing and controlling the angle of a 1-D helicopter beam. The system estimates beam angle using accelerometer and gyroscope data from an MPU6050 IMU and drives a motor using PWM output.

A VGA display provides live visualization of:

* Beam angle
* Angle error
* Motor PWM command

Runtime serial input allows live adjustment of:

* Desired beam angle
* PID proportional gain (Kp)
* PID integral gain (Ki)
* PID derivative gain (Kd)

The project was inspired by Van Hunter Adams’ RP2040 helicopter control demonstrations.

---

# Features

* Real-time PID control loop
* Complementary filter for IMU sensor fusion
* 1 kHz interrupt-driven control system
* VGA graphics visualization
* Runtime serial PID tuning
* Dual-core RP2040 task separation
* PWM motor control
* Fixed-point arithmetic optimization
* I2C communication with MPU6050

---

# Hardware Used

* Raspberry Pi Pico / RP2040
* MPU6050 IMU
* VGA monitor
* DC motor / helicopter test rig
* External motor driver

---

# Hardware Connections

## VGA

| RP2040 GPIO | VGA          |
| ----------- | ------------ |
| GPIO 0      | HSync        |
| GPIO 1      | VSync        |
| GPIO 2      | Green (470Ω) |
| GPIO 3      | Green (330Ω) |
| GPIO 4      | Blue (330Ω)  |
| GPIO 5      | Red (330Ω)   |
| GND         | VGA GND      |

## MPU6050

| RP2040 GPIO | MPU6050 |
| ----------- | ------- |
| GPIO 26     | SDA     |
| GPIO 27     | SCL     |
| 3.3V        | VCC     |
| GND         | GND     |

## PWM Output

| RP2040 GPIO | Function         |
| ----------- | ---------------- |
| GPIO 14     | PWM Motor Output |

---

# Software Architecture

## Core 0

* Serial input
* PID parameter adjustment

## Core 1

* VGA graphics rendering

## PWM Interrupt Service Routine

* IMU sampling
* Complementary filtering
* PID calculations
* PWM motor updates

---

# PID Controller

The controller implements:

* Proportional term
* Integral term
* Derivative term

The beam angle is estimated using a complementary filter combining accelerometer and gyroscope measurements.

---

# VGA Visualization

The VGA display shows:

## Top Graph

* Real-time beam angle (0°–130°)

## Bottom Graph

* PWM motor command signal

Additional on-screen text displays:

* Current beam angle
* Angle error

---

# Building and Running

## Requirements

* Pico SDK
* Cornell Protothreads Library
* VGA graphics library
* MPU6050 driver

## Build

```bash
mkdir build
cd build
cmake ..
make
```

Flash the generated UF2 file to the Raspberry Pi Pico.

---

# Demo

Example runtime serial menu:

```text
Select parameter to change:
1 -> Desired Angle
2 -> Kp
3 -> Ki
4 -> Kd
q -> Quit
```

---

# Future Improvements

* Anti-windup protection
* Derivative filtering
* Better PWM saturation handling
* Advanced state estimation
* Closed-loop speed control
* Wireless parameter tuning

---

# Acknowledgments

Inspired by the RP2040 control systems projects and tutorials by Van Hunter Adams.
