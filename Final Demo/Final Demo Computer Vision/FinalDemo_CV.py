""" 
Taran Soden (CV, Team 12)
EENG 350 - Seed Lab (Fall 2021)
Final Demo Computer Vision

Requires: RemoveCameraDistortion.py

"""

# -- Inclusions --
from picamera.array import PiRGBArray
from picamera import PiCamera
import RemoveCameraDistortion as rcd  # CV created script
import numpy as np
import cv2 as cv
import time

import smbus2
import board
import busio


# -- Global Variables --
recentPhi = 0.0
recentDir = 0b01
imwidth = 0
imheight = 0

ARDUINO_ADDRESS = 0x04
bus = smbus2.SMBus(1)

# --- Function definitions ---
'''
Name:      sendSecondary()
Function:  Send data from the secondary camera to the arduino
'''
def sendSecondary(direction, inFrame, angle):
    if angle < 0:
        absPhi = abs(angle)
        sign = 1
    else:
        absPhi = angle
        sign = 0
        
    print("Angle: %d" % round(angle))
    print("Direction: %d" % direction)
    
    array = [direction, sign, round(absPhi)
    try:
        bus.write_i2c_block_data(ARDUINO_ADDRESS, 0, array)
    except:
        print("I2C connection failed, please check connection")


'''
Name:      calibrate()
Function:  Initalize the PyCamera and fix the white balance
Input:     None
Output:    None
'''
def calibrate(camera):
    camera.resolution = (640, 480) # Set resolution
    camera.iso = 100               # Fix iso (100/200 for light, 300/400 for dark)
    camera.framerate = 30          # Fix framerate
    time.sleep(3)                  # Allow camera to adjust
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'   # Auto exposure off
    g = camera.awb_gains
    camera.awb_mode = 'off'        # Auto white balance off
    camera.awb_gains = g
    
    
'''
Name:      filter_color()
Function:  Filter any unwanted colors from the image
Input:     Original image
Output:    Color masked image
'''
def filter_color(fullImage):
    imgHSV = cv.cvtColor(fullImage, cv.COLOR_BGR2HSV)                            # Reformat image to HSV
    mask = cv.inRange(imgHSV, np.array([95,100,5]), np.array([125,255,255]))     # Filter blue
    maskedHSV = cv.bitwise_and(imgHSV, imgHSV, mask = mask)                      # Apply mask
    maskedImage = cv.cvtColor(maskedHSV, cv.COLOR_HSV2BGR)                       # Convert back to BGR
    return maskedImage


'''
Name:      detect_shape()
Function:  Detect the marker in the image and display the image
Input:     Masked image, original frame
Output:    Original frame with/without contours
'''
def detect_shape(maskedImage, originalImage):
    grayMasked = cv.cvtColor(maskedImage, cv.COLOR_BGR2GRAY)     # Convert to gray
    blurred = cv.GaussianBlur(grayMasked, (5,5), 0)              # Blur
    _, thresh = cv.threshold(blurred, 5, 255, cv.THRESH_BINARY)  # Make threshold    
    (_, contours, _) = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE) # Find any shape contours
    angle = 0
    follow = True
    
    if len(contours) > 0 :
        c = max(contours, key = cv.contourArea) # Find the biggest contour
        M = cv.moments(c)
        
        if M['m00'] > 20: # Only use the maximum contour if the area is substantial
            
            perimeter = cv.arcLength(c, True) # approximate the shape
            approx = cv.approxPolyDP(c, 0.01*perimeter, True)
            
            # If a 12 sided shape is detected (a cross) then send the command to full stop the robot
            if len(approx) == 12:
                sendSecondary(4, True, angle)
             
            # Find tape points
            bottom = tuple(c[c[:,:,1].argmax()][0])
            top = tuple(c[c[:, :, 1].argmin()][0])
            cx = int(top[0])
            cy = int(top[1])
            
            cv.drawContours(originalImage, [c], 0, (0,0,255), 5) # Draw contour
             
            # Determine the angle of the tape to send
            lastPhi = recentPhi
            angle = det_angle(cx, imwidth)
            
            global recentDir
            if angle != lastPhi:
                # Determine direction to travel to follow the line
                if cx >= (3/5)*imwidth:
                    # | | | |x|x|   Turn right
                    recentDir = 0b11
                    sendSecondary(0b11, True, angle)
                elif (cx <= (3/5)*imwidth) and (cx > (2/5)*imwidth):
                    # | | |x| | |   Go straight
                    recentDir = 0b11
                    sendSecondary(0b11, True, angle)
                elif cx < (2/5)*imwidth:
                    # |x|x| | | |   Turn left
                    recentDir = 0b11
                    sendSecondary(0b11, True, angle)
             
        elif recentDir != 0b00:
            recentDir = 0b00
            sendSecondary(0b00, False, angle)
            
    elif recentDir != 0b00:
        recentDir = 0b00
        sendSecondary(0b00, False, angle)

    return originalImage

        
'''
Name:      det_angle()
Function:  Determine the angle the marker is relative to the center of the image
Input:     x position of point(cx), image width (imwidth)
Output:    None

Key:       + left | - right
'''
def det_angle(cx, imwidth):
    midlineWidth  = 0.5 * imwidth # Determine the vertical midline of image
    if cx > midlineWidth: phi = -1 * ((53.5/2) * (cx - (midlineWidth))) / (midlineWidth) # Right side
    else: phi = ((53.5/2) * ((midlineWidth)-cx)) / (midlineWidth)                        # Left side

    global recentPhi
    if not (round(phi,2) <= recentPhi + 1 and round(phi,2) >= recentPhi - 1):
        #print("Phi: " + str(phi))
        recentPhi = round(phi,2)  #Update global angle tracker
    
    return recentPhi
        
        
# --- Main ---
camera = PiCamera() # Initialize PyCamera and calibrate
raw = PiRGBArray(camera)
calibrate(camera)
sendSecondary(0b00, False, 0)

for frame in camera.capture_continuous(raw, format="bgr", use_video_port=True):
    
    image = frame.array # Get next frame
    image, imwidth, imheight = rcd.remove_distortion(image, None) # Remove distortion with camera matrix
        
    masked = filter_color(image) # Filter the image for color
    detectedImage = detect_shape(masked, image)
    cv.imshow('Camera input', detectedImage)
    
    raw.truncate(0)               # Clear capture stream for next picture
    if cv.waitKey(1) == ord('q'): # If 'q' input stop capture
        break

cv.destroyAllWindows() # Destroy windows if quit
