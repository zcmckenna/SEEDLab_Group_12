""" 
Taran Soden (CV, Team 12)
EENG 350 - Seed Lab Fall 2021
Mini Project Computer Vision
"""

from picamera import PiCamera
from time import sleep
import numpy as np
import cv2 as cv
import smbus2
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Global Variables
recentQuad = 0
ARDUINO_ADDRESS = 0x04
lcd_i2c = busio.I2C(board.SCL, board.SDA)
lcd = character_lcd.Character_LCD_RGB_I2C(lcd_i2c, 16, 2)
bus = smbus2.SMBus(1)


"""
Function: sendQuad()
Purpose:  Send the current quadrant over I2C
* Provided by Zach McKenna
"""
def sendQuad(quadNum):
    quadNum = int(quadNum)
    if quadNum == 1 or 2 or 3 or 4: # marker is in the top left corner
        try:
            bus.write_byte_data(ARDUINO_ADDRESS, 0, quadNum)
        except:
            print("I2C connection failed, please check connection")
        lcd.color = [0, 100, 0]
        lcd.message = "Current Quad:\n" + str(quadNum)

    else:
        print("Quadrant number: " + str(quadNum) + " was not recognized\n")

"""
Function: calibrate()
Purpose:  Caibrate the Py Camera and fix the white balance
"""
def calibrate(cap):
    avgWB = 0
    for i in range(21): # capture 20 images and determine the average white balance range
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame. Exiting ...")
            break        
    
    
"""
Function: det_quad()
Purpose:  Determine what quadrant the marker is in

Key:
 ================
| Quad 2  Quad 1 |
| Quad 3  Quad 4 |
 ================

"""    
def det_quad(cx, cy, imheight, imwidth):
    # determine midlines
    midlineWidth  = 0.5 * imwidth
    midlineHeight = 0.5 * imheight
    
    # determine quadrant
    if cx > midlineWidth and cy < midlineHeight:
        quad = 1
    elif cx < midlineWidth and cy < midlineHeight:
        quad = 2
    elif cx < midlineWidth and cy > midlineHeight:
        quad = 3
    else:
        quad = 4
        
    # update the recent quadrant and send over I2C
    global recentQuad
    if quad != recentQuad:
        recentQuad = quad
        print(recentQuad)
        #sendQuad(recentQuad)


"""
Function: marker_capture()
Purpose:  Detect the marker in the video frame and determine the marker location
"""
def marker_capture(cap):
    ret, frame = cap.read()
    imheight = frame.shape[0] # get image size
    imwidth  = frame.shape[1]
    
    while True:
        ret, frame = cap.read() # capture frame
        if not ret: # if frame is read correctly ret is True
            print("Can't receive frame. Exiting ...")
            break
    
        # Step 1: Color filter
        imgHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV) # reformat image to HSV
        
        # create a mask for the detected color and apply to the frame
        mask = cv.inRange(imgHSV, np.array([80,50,0]), np.array([145,255,255])) # HSV boundaires for blue
        maskedHSV = cv.bitwise_and(imgHSV, imgHSV, mask = mask)
        
        #  Step 2: Shape filter
        # make the masked image grayscale and threshold the image
        masked = cv.cvtColor(maskedHSV, cv.COLOR_HSV2BGR)
        grayMasked = cv.cvtColor(masked, cv.COLOR_BGR2GRAY)
        blurred = cv.GaussianBlur(grayMasked, (5,5), 0)
        _, thresh = cv.threshold(blurred, 80, 255, cv.THRESH_BINARY)
        
        # find any shape contours
        (_, contours, _) = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        markerExist = False
        
        # check all found contours for the marker
        for c in contours:    
            perimeter = cv.arcLength(c, True) # approximate the shape
            approx = cv.approxPolyDP(c, 0.01*perimeter, True)
            
            # determine if the marker is present
            if len(approx) == 4:
                markerExist = True
                M = cv.moments(c)
                
                # if contour area is less than 250, don't count the contour as a marker and don't calculate position
                if M['m00'] > 250:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv.drawContours(frame, [c], 0, (0,0,255), 5)
                    det_quad(cx, cy, imheight, imwidth) # determine the quadrant
                   
        # display the resulting frame, press 'q' to quit capture
        cv.imshow('Camera input', frame)
        if cv.waitKey(1) == ord('q'):
            break
        
    # release the capture to quit
    cap.release()
    cv.destroyAllWindows()
    
# --------- Main --------
# init video capture
cap = cv.VideoCapture(0)
sleep(2) # wait for camera adjust

if not cap.isOpened():
    print("Cannot open camera")
    exit()
    
calibrate(cap)
marker_capture(cap)