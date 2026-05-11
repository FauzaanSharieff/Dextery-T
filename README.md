# Dextery-T

Dextery-T is a 6-DOF articulated robotic arm project built using MG996R servo motors, an Arduino Uno, and a PCA9685 PWM servo driver. The goal of the project is to control the robot’s TCP/tool center point in Cartesian coordinates, allowing the arm to move to desired `(x, y, z)` positions using inverse kinematics.

The project is currently in the development and testing phase, with the main focus on validating the kinematic model, inverse kinematics, workspace limits, and Arduino-based servo control.

---

## Project Goal

The final goal of Dextery-T is to create a small robotic arm capable of receiving Cartesian position commands and converting them into synchronized joint movements.

The intended control flow is:

Target Cartesian Position
        ↓
Inverse Kinematics
        ↓
Joint Angle Calculation
        ↓
Synchronized Servo Motion
        ↓
Robot Reaches Desired TCP Position

At the current stage, the project focuses on testing the mathematical model and validating movement behavior before moving toward more advanced control interfaces.

---

## Current Development Status

Dextery-T has already passed the basic joint-space movement stage. The robot can move its servos synchronously to specified joint angle targets.

Cartesian control has also been implemented and tested successfully. The robot can now receive a target Cartesian position, calculate the required joint angles through inverse kinematics, and move the TCP/tool center point to the desired position. The current implementation assumes a fixed TCP orientation, where the tool/end-effector faces downward. Because of this, only the desired Cartesian position needs to be provided, while the orientation is handled internally by the kinematic model.

The current development stage focuses on improving the quality, safety, and flexibility of the movement. This includes:

- Adding a proper controller/input system for commanding Cartesian motion
- Implementing movement interruption or safer stopping behavior
- Improving the trajectory generation method
- Fixing the current cubic interpolation issue, where every small interpolated Cartesian path segment starts and ends with zero velocity
- Reducing uneven stop-start motion between interpolated Cartesian points
- Adding dynamic speed control instead of relying on fixed movement durations
- Further testing Cartesian paths on the physical robot

At this stage, the main kinematic functionality is working. The focus is now shifting from “can the robot reach the Cartesian target?” to “can the robot move there smoothly, safely, and controllably?”

---

## Repository Contents

### `dextery_t_main.ino`

This is the main Arduino file for controlling the robot arm.

It contains the embedded implementation used to drive the servo motors through the PCA9685 PWM driver. The code is responsible for:

- Communicating with the PCA9685 servo driver
- Sending angle commands to the motors
- Executing synchronized joint movement
- Converting inverse kinematics results into servo target angles
- Testing Cartesian movement on the physical robot

The Arduino code is the main bridge between the mathematical model and the real hardware.

---

### `kinematicsSetup.m`

This MATLAB script sets up the kinematic model of the robot.

It defines the robot’s geometry, joint variables, transformation matrices, and the relationships between the different coordinate frames of the arm.

This file is mainly used to derive and verify the forward kinematics of the system. It forms the mathematical foundation for the rest of the project.

---

### `inverseKinematics.m`

This MATLAB script implements the inverse kinematics of the robotic arm.

Using the already-defined transformation model, it calculates the joint angles required for the TCP to reach a given Cartesian position.

The script is used to test and validate the IK equations before transferring the logic into the Arduino `.ino` file.

---

### `cartesianLimits.m`

This MATLAB script is used to explore the reachable Cartesian workspace of the robot.

It calculates and visualizes the position limits of the TCP based on the robot’s link lengths, joint limits, and kinematic structure.

The resulting workspace resembles a torus-like reachable region, helping identify which Cartesian points are physically reachable by the arm.

This is important because the inverse kinematics should only be tested with positions that the robot can actually reach.

---

### `DH New.jpeg`

This image shows the robot configuration with the coordinate origins, frame transformations, and kinematic structure visualized.

It is used as a reference for understanding how the coordinate frames are assigned and how the robot’s transformations are built.

---

## Hardware Used

The current hardware setup includes:

- Arduino Uno
- PCA9685 PWM servo driver
- MG996R servo motors
- External battery supply
- Voltage regulator for servo power
- Serial/USB connection for programming and debugging


---

## Software Used

The project currently uses:

- Arduino IDE
- MATLAB
- PCA9685 servo driver library
- Serial monitor for debugging
- Git/GitHub for version control

MATLAB is used for mathematical development, simulation, and validation, while Arduino is used for real-time control of the physical robot.

---

## Development Phases

### Phase 1: Joint-Space Control

Status: Mostly complete.

In this phase, the robot is controlled by directly providing target joint angles. The servos move synchronously from their current positions to the desired joint configuration.

This phase confirms that the motors, servo driver, power system, and basic movement functions work correctly.

---

### Phase 2: Cartesian Control

Status: In progress.

In this phase, the robot receives Cartesian position targets instead of direct joint angles.

The inverse kinematics calculates the required joint angles, and the Arduino code commands the servos accordingly.

Although the robot is now capable of reaching specific targets in cartesian coordinates, this feature is yet to be implemented using a controller (with joysticks probably).

This is the current main development phase of Dextery-T.


---

## Notes and Current Limitations

The current trajectory interpolation uses cubic motion profiles where velocity starts and ends at zero. This works well for single complete movements, but when a path is split into many small Cartesian steps, each step starts and ends with zero velocity. This can lead to unnecessarily stop-start motion.

A future improvement will be to replace this with a smoother velocity profile that allows continuous motion across path segments.

Additionally, the current implementation assumes a fixed TCP orientation. This simplifies the inverse kinematics and is suitable for tasks such as basic pick-and-place movement, but full orientation control may be added later.

---

## Project Vision

Dextery-T is intended to be more than just a basic servo arm. The long-term vision is to create a functional robotic platform that can move meaningfully in Cartesian space and eventually be connected to more advanced control systems.

The project combines mechanical design, embedded programming, kinematics, control engineering, and simulation into one practical robotics system.

As a robotics student, when someone asks me how many robots I have already made, I would like the answer to gain me some aura.