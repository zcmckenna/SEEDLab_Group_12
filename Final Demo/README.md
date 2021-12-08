# Demo 2, SEED Lab Group 12, Fall 2021
* Charles James - Simulation and Control
* Zach McKenna - System Integration
* Brandon Nahsjun - Localization
* Taran Soden - Computer Vision

### Purpose
The purpose of this demo is to integrate all subsystems in order to allow the robot to line follow as well as make the right hand turns for the final demo. Computer vision and localization subsystems continually communicate in this demo.  

### Contents
* `Final Demo Computer Vision/` - The directory containing all the computer vision code. 
  * `Final Demo Computer Vision/FinalDemoCV.py` - The python code that detects a piece of tape in the camera and sends feedback on the tape's position in frame to the Arduino.
  * `Computer Vision/RemoveCameraDistortion.py` - The python code that holds the determined camera matrix and applies the matrix to a camera frame.
* `robotCode/robotCode.ino` - The Arduino code with the PI controllers that handles all the motor code for the project.
* `Simulation and Control/` - The directory containing all the simulation and control models for the final demo / demo 2.