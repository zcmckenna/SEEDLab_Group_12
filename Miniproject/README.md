# Mini Project, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of the mini project is to get all the team members familiar with the tools and techniques that they will need to know to complete the final project. This project involved OpenCV to detect a marker and its location in frame, encoders to keep track of a motors position, fine motor control with a motor driver, Simulink simulation to create a PI controller, Arduino code to implement a PI controller, and I2C communication to pass data between a Raspberry Pi and an Arduino.

### Contents
* `pi_control_arduino_code/pi_control_arduino_code.ino` - The main code that runs on the Arduino. This includes motor control, encoder logic, PI control, and I2C communication.
* `Mini Project CV.py` - The main code that runs on the Raspberry Pi. This includes the OpenCV logic and I2C communication logic.
* `Simulink_Matlab/` - The directory containing the Matlab live script for plotting, the Simulink model, and the Arduino code for testing the open loop and closed loop response.