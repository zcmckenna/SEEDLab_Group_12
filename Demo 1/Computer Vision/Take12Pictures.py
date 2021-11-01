""" 
Taran Soden (CV, Team 12)
EENG 350 - Seed Lab (Fall 2021)
Take 12 Pictures

Function: Takes 12 pictures in a row at a set calibration
Photos used by CameraMatrixDetect.py
"""

from picamera import PiCamera
from time import sleep
import numpy as np
import cv2 as cv
import glob

'''
Name:      calibration()
Function:  Initalize the PyCamera and fix the white balance
Input:     None
Output:    None
'''
def calibrate(camera):
    camera.resolution = (640, 480) # set resolution
    camera.framerate = 30 # fix framerate
    camera.iso = 300 # fix iso (100/200 for light, 300/400 for dark)
    sleep(3) # allow camera to adjust
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off' # auto exposure off
    g = camera.awb_gains
    camera.awb_mode = 'off' # auto white balance off
    camera.awb_gains = g

# ---- take images ----
camera = PiCamera()
calibrate(camera)
camera.rotation = 180
camera.start_preview(alpha=200)
for i in range(12):
    sleep(5)
    print("Pic taken")
    camera.capture('image%s.jpg' % i)
camera.stop_preview()