# Demo 2, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of this demo is to integrate all subsystems in order to allow the robot to line follow for the final demo. Computer vision and localization subsystems continually communicate in this demo.  

### Contents
* `Computer Vision/` - The directory containing all the computer vision code. 
  * `Computer Vision/Demo2_ComputerVision.py` - The python code that detects a piece of tape in the camera and sends feedback on the tape's position in frame to the Arduino.
  * `Computer Vision/RemoveCameraDistortion.py` - The python code that holds the determined camera matrix and applies the matrix to a camera frame.
####* `Localization/` - The directory containing all the code for localization.
####  * `Localization/localizationCalculations.c` - The c code that runs on the Arduino, it includes the encoder code, PID control, and velocity calculations.
####  * `Localization/stepRespCode.c` - The c code that runs on the Arduino, it contains the code to run the step response experiments.
* `Simulation and Control` - The directory containing all the simulation and control models for demo 2.
