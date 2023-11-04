# Arduino Robot Controller

This repository contains an Arduino sketch for controlling a robot equipped with DC motors and servos.

## Project Overview

The sketch provides functionality for a robot that can navigate through its environment, avoid obstacles, and perform actions like picking up and placing objects. It employs PID control for precise movement and integrates ultrasonic sensors for distance measurement.

## Features

- Motor control for movement and navigation.
- Servo control for object manipulation.
- Ultrasonic distance measurements for obstacle avoidance.
- PID control for precise operation.
- State machine logic for behavior management.

## Hardware Requirements

- Arduino compatible microcontroller.
- DC motors with H-bridge motor driver.
- Servo motors.
- Ultrasonic distance sensors.

## Software Requirements

- Arduino IDE for uploading the sketch to the microcontroller.
- Servo library for Arduino (included by default in the Arduino IDE).

## Getting Started

To use this code, follow these steps:

1. Connect the hardware components as per the circuit diagram provided in the `schematics` directory.
2. Open the `RobotController.ino` file in the Arduino IDE.
3. Install any necessary libraries using the Library Manager in the Arduino IDE.
4. Upload the sketch to your Arduino board.
5. Power on your robot to begin autonomous navigation.

## Configuration

Tune the PID constants `kp`, `ki`, and `kd` in the sketch to match the specific characteristics of your robot.

## Contributing

If you'd like to contribute, please fork the repository and use a feature branch. Pull requests are warmly welcome.

## Licensing

This project is released under the MIT License.
