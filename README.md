# Crane Arm with Sensor Integration

This repository contains the code and basic instructions for assembling a crane arm that uses a stepper motor for rotation, a servo motor for the claw, and several sensors to enhance safety and functionality.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [How It Works](#how-it-works)
- [Troubleshooting--Notes](#troubleshooting--notes)
- [Credits](#credits)

## Overview

The idea behind this project was to build a small-scale crane arm that can be controlled via joystick and IR remote while responding to various sensors for safety and environmental monitoring. The crane’s main movement is powered by a stepper motor, which provides precise rotation, and the claw is controlled by a servo motor.

## Features

- **Stepper Motor Arm Control:** Precise positioning of the crane arm.
- **Servo Motor Claw:** Enables gripping and releasing items.
- **Multiple Sensors:**
  - PIR Sensor for motion detection.
  - Ultrasonic Sensor for obstacle detection at close range.
  - MPU6050 for tilt/pitch measurements.
  - DHT Sensor for temperature/humidity checks.
  - Water Detection sensor for emergency shutdown if water is present.
  - IR Receiver for remote control operations.
- **LCD Display:** Shows real-time system status (mode, sensor readings, etc.).
- **Joystick:** Allows manual control, switching between stepper and servo modes.
- **Automatic Safety Routines:** Emergency stops and resets if obstacles or water are detected or if environmental thresholds are exceeded.

## Hardware Setup

**Main Components:**
- Microcontroller (e.g., Arduino)
- Stepper Motor + Driver (or driver shield like AccelStepper-compatible pins)
- Servo Motor (for the claw)
- 74HC595 Shift Register (for the LCD)
- LCD Screen (16x2 or similar)
- PIR Sensor
- Ultrasonic Sensor
- MPU6050 (Accelerometer + Gyro)
- DHT11 or DHT22 sensor (Temperature/Humidity)
- Water Sensor
- IR Receiver
- Joystick (potentiometers + switch)

**Wiring Considerations:**
- Use the correct pin assignments as defined in the code.
- Check power requirements for the stepper and servo (use separate power if needed).
- Verify sensor voltage levels (5V vs. 3.3V).
- The 74HC595 offloads some pins for the LCD.

**Physical Assembly:**
- Mount the stepper motor securely for arm rotation.
- Attach the servo motor to the claw/gripper.
- Position sensors optimally (e.g., ultrasonic sensor facing forward).
- Use sturdy wiring and proper cable management.

## Software Setup

**Libraries Needed:**
- `Wire.h`
- `MPU6050.h`
- `AccelStepper.h`
- `Servo.h`
- `IRremote.h`
- `DHT.h`
- Plus any default libraries like `math.h`

**Installation:**
- Install these libraries via the Arduino Library Manager or from the component manufacturer’s GitHub pages.

**Compiling & Uploading:**
- Open the `.ino` file in the Arduino IDE.
- Select the correct board and port.
- Compile and upload the code.

## How It Works

**Initialization:**
- Initializes each sensor and motor.
- Calibrates the MPU6050 for stable angle readings.
- Resets the stepper position.

**Main Loop:**
- Regularly reads sensors (gyro/accelerometer, ultrasonic, PIR, DHT, water sensor).
- Checks for IR remote signals to toggle power or adjust speed.
- Switches between servo mode (claw control) and stepper mode (arm rotation) via joystick.
- Updates the LCD with real-time info (mode, angles, distances).
- Automatically stops or moves to a safe “home” position if dangerous conditions are detected.

**Controls:**
- **Joystick:** Tilt left/right to move the servo or stepper, depending on the active mode.
- **IR Remote:** Toggle power, adjust stepper speed, etc.
- **Automatic:** Sensors override manual controls during emergencies.

## Troubleshooting--Notes

- If the LCD isn’t displaying text, verify the shift register and LCD connections.
- For irregular servo movements, check the power supply; servos may need a separate regulated source.
- If the stepper jitters or misbehaves, recheck the wiring sequence on the driver pins.
- Ensure the DHT sensor is reading valid data; a pull-up resistor might be needed on the data line.
- Keep the MPU6050 steady during calibration.

## Credits

- Portions of the code reference official documentation for each component’s library (e.g., MPU6050, AccelStepper, IRremote, DHT).
- This project was inspired by various open-source examples and manufacturer guides, but heavily modified for this setup.
- Wiring diagrams and logic are original to this project, reflecting significant hands-on effort in building the crane mechanism and ensuring safe operation.
