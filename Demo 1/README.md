# Demo 1, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of this demo is to start to merge and integrate the different subsystems together. The localization and simulation and control parts are integrated together and the computer vision and system integration parts are integrated together.

### Contents
* `Computer Vision/` - The directory containing all the computer vision code. 
* `Localization/` - The directory containing all the code for localization.
* `Simulation and Control` - The directory containing all the simulation and control models for demo 1.
* `Localization/localizationCalculations.c` - The c code that runs on the Arduino, it includes the encoder code, PID control, and velocity calculations.
* `Localization/stepRespCode.c` - The c code that runs on the Arduino, it contains the code to run the step response experiments.
