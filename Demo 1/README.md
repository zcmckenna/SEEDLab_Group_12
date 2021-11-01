# Demo 1, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of this demo is to start to merge and integrate the different subsystems together. The localization and simulation and control parts are integrated together and the computer vision and system integration parts are integrated together.

### Contents
* `Documentation/` - The directory containing all project documentation. 
* `pi_control_arduino_code/pi_control_arduino_code.ino` - The main code that runs on the Arduino. This includes motor control, encoder logic, PI control, and I2C communication.
* `Mini Project CV.py` - The main code that runs on the Raspberry Pi. This includes the OpenCV logic and I2C communication logic.
* `Simulink_Matlab/` - The directory containing the Matlab live script for plotting, the Simulink model, and the Arduino code for testing the open loop and closed loop response.
