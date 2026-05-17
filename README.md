# Dextery-T

Dextery-T is a 5-DOF articulated robotic arm project built using MG996R servo motors, an Arduino Uno, and a PCA9685 PWM servo driver.

The goal of the project is to control the robot's TCP/tool center point in Cartesian coordinates, allowing the arm to move along the x, y, and z axes while maintaining a fixed end-effector orientation.

## Project Status

Dextery-T has been successfully completed.

The main goal of the project has been achieved: the robot can move its TCP in Cartesian space using inverse kinematics while keeping the end-effector orientation fixed. The project also includes a controller interface and a Python serial bridge for sending movement commands to the robot.

## Project Goal

The goal of Dextery-T is to create a small robotic arm capable of receiving Cartesian movement commands and converting them into synchronized servo motion.

The general control flow is:

1. Receive Cartesian movement command
2. Calculate a reachable Cartesian target
3. Use inverse kinematics to calculate the required joint angles
4. Move all joints synchronously
5. Update the TCP position using forward kinematics

## Features

- 5-DOF robotic arm architecture
- Cartesian TCP control along translational axes
- Fixed downward-facing end-effector orientation
- Inverse kinematics implemented
- Forward kinematics for position tracking
- Synchronous multi-servo motion
- Continuous Path cartesian interpolation for straight trajectory
- Linear joint interpolation for smoother movement
- Workspace boundary calculation using a torus-like reachable region
- Controller-based movement commands
- PySerial communication bridge between controller and robot Arduino
- Gripper open/close control during idle state
- Movement interruption through idle command handling

## Hardware Used

- Arduino Uno
- PCA9685 PWM servo driver
- MG996R servo motors
- External battery supply (2S LiPo)
- Voltage regulator for servo power
- Controller Arduino
- Joystick/controller input system
- USB serial connection for communication and debugging

## Software and Tools

- Arduino IDE
- MATLAB
- Python
- PySerial
- Git/GitHub
- PCA9685 servo driver library

## Concepts Involved

### Forward Kinematics

Forward kinematics is used to calculate the current TCP position based on the robot's joint angles. This allows the software to track where the end-effector is after each movement.

### Inverse Kinematics

Inverse kinematics is used to calculate the joint angles required to move the TCP to a desired Cartesian position. The implementation assumes a fixed end-effector orientation, so only the Cartesian position needs to be commanded.

### Cartesian Control

Instead of directly commanding individual joint angles, Dextery-T allows the TCP to be moved along Cartesian directions such as `+x`, `-x`, `+y`, `-y`, `+z`, and `-z`.

### Synchronous Joint Motion

All servo motors are moved together over the same movement duration. This allows different joints to travel different angle distances while still starting and ending their motion at the same time.

### Linear Interpolation

Linear interpolation is used to gradually move each servo from its current joint angle to its target joint angle. This avoids sudden jumps in servo commands while keeping the movement logic simple and predictable.

### Workspace Limiting

The reachable Cartesian workspace is modeled as a torus-like region. This helps determine valid target positions and prevents the robot from attempting to move outside its physical reach.

### Serial Communication

A Python PySerial bridge is used to relay commands from a controller Arduino to the robot Arduino. The bridge validates movement commands and forwards them to the main robot controller over USB serial.

### Embedded Servo Control

The Arduino Uno communicates with the PCA9685 PWM driver to generate servo PWM signals. Joint angles are converted into PWM values before being sent to the corresponding servo channels.

## Repository Structure

### `dextery_t_main.ino`

Main Arduino program for the robotic arm. It contains the embedded implementation of servo control, inverse kinematics, forward kinematics, Cartesian movement, workspace limits, gripper control, and command handling.

### `dextery_t_controller.ino`

Arduino controller program used to read joystick and switch inputs. It converts user input into high-level movement commands such as `+x`, `-z`, `idle,o`, and `idle,c`.

### `serial_bridge.py`

Python script that acts as a serial communication bridge between the controller Arduino and the robot Arduino. It reads commands from the controller, validates them, and forwards them to the main robot.

### `kinematicsSetup.m`

MATLAB script used to define and test the robot's kinematic chain, transformation matrices, and coordinate frame relationships.

### `inverseKinematics.m`

MATLAB script used to develop and validate the inverse kinematics equations before transferring the logic to the Arduino implementation.

### `cartesianLimits.m`

MATLAB script used to analyze and visualize the reachable Cartesian workspace of the robot arm.

### `Origin Transformations.jpeg`

Reference image showing the robot's coordinate frames along each joint.

## Command Interface

Dextery-T receives simple text-based movement commands. These commands are generated by the controller Arduino and passed to the robot Arduino through the Python serial bridge.

| Command | Description |
|---|---|
| `+x` | Move TCP in the positive x direction |
| `-x` | Move TCP in the negative x direction |
| `+y` | Move TCP in the positive y direction |
| `-y` | Move TCP in the negative y direction |
| `+z` | Move TCP in the positive z direction |
| `-z` | Move TCP in the negative z direction |
| `idle,o` | Idle state with gripper open |
| `idle,c` | Idle state with gripper closed |

## Development Phases

### Phase 1: Joint-Space Control

The first phase focused on controlling the servo motors directly using target joint angles. This confirmed that the motors, PWM driver, power system, and basic movement logic worked correctly.

### Phase 2: Cartesian Control

The second phase implemented Cartesian control of the TCP. Desired Cartesian positions are converted into joint angles using inverse kinematics and then sent to the motors through synchronized servo movement.

### Phase 3: Controller Integration

The robot can be commanded using a separate controller Arduino. A Python PySerial bridge forwards the controller commands to the main robot Arduino.

## Limitations

- The current inverse kinematics implementation assumes a fixed end-effector orientation. This is suitable for the current goal of Cartesian translation and PnP, but full orientation control is not implemented. Due to the robot having 5 degrees of freedom in joint space, it is only possible to control orientation along 2 axes.
- The robot uses hobby servo motors. So precision, torque, and repeatability are limited compared to industrial robotic arms.
- The workspace is limited by the physical geometry and servo range of the arm.

## Future Work (maybe)

- Improve mechanical rigidity and repeatability
- Add more advanced trajectory planning
- Improve gripper mechanics and object handling using tactile sensing
- Integrate Dextery-T with external control systems
- Implement autonomous movement based on object detection
- Use the robot as a modular platform for future projects

## Project Vision

Dextery-T was built as a practical robotics platform combining mechanical design, embedded programming, kinematics, control logic, MATLAB-based modeling, and serial communication - with the purpose of helping me get deeply familiar with robot kinematics concepts.

Its modular software structure allows access to both high-level Cartesian control and low-level servo control, making it suitable for future extensions and integration with other robotics concepts.

As a robotics student, when someone asks me how many robots I have made so far, I would like the answer to make me feel a bit better about myself.