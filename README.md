# Arboreal Ascenders: Tree-Climbing Robot for Wildfire Suppression

This repository contains the full documentation and source code for **Arboreal Ascenders**, a senior design project completed at Stevens Institute of Technology. The goal was to create an autonomous tree-climbing robot capable of deploying fire suppressants from elevated positions to slow or contain the spread of forest fires.

---

## My Role

As part of a five-member team, I was responsible for:
- Designing and wiring the robot’s electrical systems
- Programming all robot behaviors in Arduino
- Developing a PI controller for DC motors with encoder feedback
- Implementing autonomous climbing logic and sensor-based navigation
- Debugging and integrating all motor, sensor, and control subsystems

---

## Final Report

👉 [Click here to view the full final report](Arboreal_Ascenders_Final_Report.pdf)

The report includes the background, design process, hardware/software architecture, testing methods, and results.

---

## Code Structure

All Arduino files for the robot and its subsystems are located in the `code/` folder.

### `code/final_robot/`
- `FinalRobotDesign.ino`: Final version of the robot's climbing logic
- `launch.json`: VS Code configuration for debugging/development

### `code/tests/`
- `DCMotorANDServoTest.ino`: Combines DC and servo control testing
- `DCMotorTuningandArmExtension.ino`: PI controller tuning and arm mechanics
- `MotorEncoderTest.ino`: Tests encoder signal reading for motors
- `ServoMotors_Spine.ino`: Servo tests for spine movement

---

## Technologies Used

- **Arduino & Raspberry Pi** communication
- **Custom PI motor controller**
- **LiDAR and barometric sensors** for path planning
- **3D-printed mechanical components** using SOLIDWORKS
- Rack and pinion system for gripping and climbing
- Closed-loop feedback and waypoint pathing

---

## Project Summary

- **Institution**: Stevens Institute of Technology  
- **Timeline**: September 2023 – May 2024  
- **Funding**: $2,000 from L3Harris  
- **Team**: 5 members  
- **Final Deliverable**: Working climbing prototype + report + test cases

---

## About

This project was created to reduce risks to firefighters and wildlife by using robotic swarms to surround and slow the spread of forest fires. This repository is public to showcase engineering and programming work to potential employers.
