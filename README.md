# ROV Control System

## Project Overview

This project implements a complete control system for a Remotely Operated Vehicle (ROV). The system consists of two main components:

1. **Autopilot** - Runs on a Raspberry Pi onboard the ROV
2. **Mission Planner** - Runs on a PC for remote control and monitoring

## System Architecture

### Autopilot (Raspberry Pi)

The onboard system handles sensor reading, motor control, and communication with the ground station.

#### Key Components:

- **Sensor Integration**:

  - MPU9250 for accelerometer and gyroscope data (pitch, roll)
  - HMC5883L magnetometer compass for heading
  - Data filtering using Kalman and low-pass filters

- **Motor Control**:

  - Automatic depth control
  - Automatic heading control
  - Manual control via joystick commands

- **Communication**:
  - MAVLink protocol for telemetry and command exchange
  - Camera video feed transmission

### Mission Planner (PC)

The ground station provides a user interface for ROV control, visualization, and monitoring.

#### Key Components:

- **User Interface**:

  - 3D visualization of ROV attitude
  - Compass and attitude indicator displays
  - Video feed from ROV camera
  - Control buttons and status displays

- **Control Systems**:

  - Joystick integration
  - Button controls for manual operations
  - Mode switching between manual and automatic control

- **Monitoring**:
  - Real-time status display
  - Log viewer with color-coded messages
  - Graphical display of attitude and other sensor readings

## Key Features

### Sensor Integration

- The system integrates various sensors including accelerometers, gyroscopes, and magnetometers
- Advanced filtering using Kalman and low-pass filters to provide stable readings
- Combined sensor fusion for accurate position and attitude tracking

### Automatic Control

- PID-based automatic depth control maintains ROV at desired depth
- Automatic heading control keeps ROV pointed in desired direction
- Control parameters are adjustable through the Mission Planner interface

### Real-time Visualization

- 3D model showing the ROV's current attitude (pitch, roll, yaw)
- Compass display for heading information
- Artificial horizon/attitude indicator for pilot reference
- Real-time graphs of sensor data

### Communication and Logging

- MAVLink protocol implementation for robust ROV-to-PC communication
- Comprehensive logging system for debugging and mission review
- Color-coded log viewer for easy identification of issues

## Setup and Installation

### Requirements

- Raspberry Pi for Autopilot system
- IMU sensors (MPU9250, HMC5883L)
- Python 3.7+ with required packages (see requirements.txt)
- PyQt5 for Mission Planner GUI

### Installation

1. Clone the repository
2. Install required packages:
