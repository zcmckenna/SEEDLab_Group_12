""" 
Taran Soden (CV, Team 12)
EENG 350 - Seed Lab (Fall 2021)
Demo 1 Computer Vision

Requires: RemoveCameraDistortion.py
"""

from picamera.array import PiRGBArray
from picamera import PiCamera
from math import trunc
import RemoveCameraDistortion as rcd  # CV created script
import numpy as np
import cv2 as cv
import time
import smbus2
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Global Variables
recentPhi = 0.0
tapeDetected = False
imwidth = 0
imheight = 0


ARDUINO_ADDRESS = 0x04
lcd_i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, 16, 2)
bus = smbus2.SMBus(1)
lcd.color = [0, 100, 0]


# --- Function definitions ---
'''
Function: sendAngle()
Purpose:  Send the current quadrant over I2C
* Template provided by Zach McKenna
'''

def sendAngle(angle, inFrame):
    if inFrame: # the tape is in frame
        lcd.clear()
        lcd.message = "Angle:\n" + str(angle)
    else:
        lcd.message = "Not detected\n         "


'''
Name:      calibrate()
Function:  Initalize the PyCamera and fix the white balance
Input:     None
Output:    None
'''
def calibrate(camera):
    camera.resolution = (640, 480) # set resolution
    camera.iso = 400 # fix iso (100/200 for light, 300/400 for dark)
    time.sleep(3) # allow camera to adjust
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off' # auto exposure off
    g = camera.awb_gains
    camera.awb_mode = 'off' # auto white balance off
    camera.awb_gains = g
    
    
'''
Name:      filter_color()
Function:  Filter any unwanted colors from the image
Input:     Original image
Output:    Color masked image
'''
def filter_color(fullImage):
    imgHSV = cv.cvtColor(fullImage, cv.COLOR_BGR2HSV) # reformat image to HSV
    mask = cv.inRange(imgHSV, np.array([95,100,20]), np.array([125,255,255])) # filter blue
    maskedHSV = cv.bitwise_and(imgHSV, imgHSV, mask = mask) # apply mask
    maskedImage = cv.cvtColor(maskedHSV, cv.COLOR_HSV2BGR) # convert back to BGR
    return maskedImage


'''
Name:      detect_shape()
Function:  Detect the marker in the image and display the image
Input:     Masked image, original frame
Output:    Original frame with/without contours
'''
def detect_shape(maskedImage, originalImage):
    grayMasked = cv.cvtColor(maskedImage, cv.COLOR_BGR2GRAY) # convert to gray
    blurred = cv.GaussianBlur(grayMasked, (5,5), 0) # blur
    _, thresh = cv.threshold(blurred, 20, 255, cv.THRESH_BINARY) # make threshold
        
    # find any shape contours
    (_, contours, _) = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    tapeDetected = False
    angle = 0
    # check all found contours for the tape
    for c in contours:    
        perimeter = cv.arcLength(c, True) # approximate the shape
        approx = cv.approxPolyDP(c, 0.01*perimeter, True)
            
        # determine if the tape is present
        if len(approx) >= 3:
            M = cv.moments(c)
                
            # if contour area is less than 20, don't count the contour as tape and don't calculate position
            if M['m00'] > 20:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv.drawContours(originalImage, [c], 0, (0,0,255), 5)
                angle = det_angle(cx, imwidth) # determine angle phi
                tapeDetected = True
    
    # If no tape has been detected in the frame, show "Not Detected" on the LCD screen
    if tapeDetected == False:
        sendAngle(angle, False)
        
    return originalImage

        
'''
Name:      det_angle()
Function:  Determine the angle the marker is relative to the center of the image
Input:     x position of point(cx), image width (imwidth)
Output:    None

Key:       + left | - right   (Whole number angle only ie. 85deg)
'''
def det_angle(cx, imwidth):
    
    midlineWidth  = 0.5 * imwidth # determine vertical midline of image
    
    if cx > midlineWidth:
        phi = -1 * ((53.5/2) * (cx - (midlineWidth))) / (midlineWidth) # Right side
    else:
        phi = ((53.5/2) * ((midlineWidth)-cx)) / (midlineWidth)        # Left side
    
    # Output angle phi and update global
    global recentPhi
    if not (round(phi,2) <= recentPhi + 0.2 and round(phi,2) >= recentPhi - 0.2):
        print("Phi: " + str(phi))
        recentPhi = round(phi,2)
        sendAngle(recentPhi, True)
    
    return recentPhi
        
        
# --- End function definitions ---


# init PyCamera and calibrate
camera = PiCamera()
raw = PiRGBArray(camera)
calibrate(camera)

for frame in camera.capture_continuous(raw, format="bgr", use_video_port=True):
    image = frame.array # get next frame
    
    # remove distortion with camera matrix
    image, imwidth, imheight = rcd.remove_distortion(image, None)
        
    masked = filter_color(image)
    detectedImage = detect_shape(masked, image)
    cv.imshow('Camera input', detectedImage)
    
    raw.truncate(0) # clear capture stream for next picture
    
    if cv.waitKey(1) == ord('q'): # if 'q' input stop capture
        break

cv.destroyAllWindows() # destroy windows if quit
