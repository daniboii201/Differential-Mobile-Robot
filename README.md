# Mobile Differential Robot
Welcome to the repository for my capstone project, a work in progress as part of my ongoing studies in the Technologies of Automation & Control class during my Bachelor's in Mechanical Engineering program.

## Overview
This project is inspired by Edison Sásig's Udemy Course titled "Mobile differential robot: Autonomous Navigation." I utilized the theoretical knowledge gained from the course to implement the project, following step-by-step instructions provided by the instructor.

## Description
This project focuses on the development and testing of a mobile differential robot, designed for autonomous navigation. The key aspects covered in the Arduino code iterations include:

1. Encoder Testing: Implementation and evaluation of encoder functionality for precise motor control.
2. Bluetooth Testing: Integration and testing of Bluetooth communication for remote control capabilities.
3. Motor Speed Testing: Validation of motor speed control mechanisms to ensure optimal performance.
4. Open Loop Control: Development and testing of open-loop control algorithms.
5. PID Lambda Control: Implementation and fine-tuning of PID control for enhanced navigation precision.
6. Fine Tuning: Iterative adjustments and optimizations to refine the robot's overall performance.
7. Final Test: Comprehensive testing to validate the integrated functionalities of the robot.

### Robot Structure
The mobile differential robot's physical structure is 3D printed using PLA, with various components sourced online. The essential components include:
- 1 Arduino Nano with USB cable (for some tests)
- 1 ESP32 Module
- 1 H-bridge L298N
- 2 Motors with quadrature encoders (140RPM Chihai Motor at 12V or similar)
- 1 Buzzer
- 1 3S LiPo Battery (Turnigy 3S 1000mAh LiPo Battery) with charger
- 1 Switch
- 1 Push Button
- 1 Resistor 1kΩ
- 1 Resistor 2kΩ
- 1 Breadboard (preferably small)
- 1 Structure for component assembly
- Cables and Jumpers
- 3D printer and 100g PLA filament
The project explores the intersection of hardware and software, utilizing a range of components to achieve autonomous functionality in a differential drive configuration. Feel free to explore the code iterations and contribute to the project's development!

## Usage 

To utilize the autonomous capabilities of the mobile differential robot, follow these steps:

1. Connection Diagram:
- A connections diagram is included in the repository to assist you in correctly wiring the components.
- Follow the diagram closely to avoid any connectivity issues.

2. Structure and Assembly:
- SolidWorks files and STL files are provided for the robot's structure and assembly.
- Adjust the parts as needed and use a 3D printer to bring the components to life.
- Ensure the proper assembly of components based on the provided files.

3. Pre-Test Consideration:
- Before deploying the code, it is crucial to conduct thorough testing of your robot.
Ensure that motor functions, encoders, and Bluetooth communication are working correctly.

4. PID Control and Fine Tuning:
- Perform PID control and fine-tuning procedures to optimize the robot's navigation and responsiveness.
- Refer to the provided documentation for detailed instructions on PID tuning.

5. Upload Arduino Code:
Upload the Arduino code named "Robot_diferencial_autonomo_con_ESP32" to your ESP32 module.

6. Optimized Use:
- Once the code is uploaded, and the robot is assembled and tested, you can enjoy the optimized autonomous functionalities of your mobile differential robot.

Feel free to explore, modify, and contribute to the project. Your feedback and improvements are highly appreciated. For any inquiries or issues, refer to the documentation or reach out to the project's community.

## Course Source
For detailed insights and the original course content, refer to the Udemy course by Edison Sásig:
https://www.udemy.com/course/robotica-con-python-y-arduino-robot-movil-diferencial/
