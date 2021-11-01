# Demo 1, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of this demo is to start to merge and integrate the different subsystems together. The localization and simulation and control parts are integrated together and the computer vision and system integration parts are integrated together.

### Contents
* `Computer Vision/` - The directory containing all the computer vision code. 
* `Computer Vision/CameraMatrixDetect.py` - The python code that was used to determine the camera distortion matrix.
* `Computer Vision/Demo1CV.py` - The python code that detects a piece of tape in the camera and determines the tape angle respective to the camera midline.
* `Computer Vision/RemoveCameraDistortion.py` - The python code that holds the determined camera matrix and applies the matrix to a camera frame.
* `Computer Vision/Take12Pictures.py` - Helper python code that takes 12 pictures in succession, used in CameraMatrixDetect.py.
* `Localization/` - The directory containing all the code for localization.
* `Localization/localizationCalculations.c` - The c code that runs on the Arduino, it includes the encoder code, PID control, and velocity calculations.
* `Localization/stepRespCode.c` - The c code that runs on the Arduino, it contains the code to run the step response experiments.
* `Simulation and Control` - The directory containing all the simulation and control models for demo 1.
