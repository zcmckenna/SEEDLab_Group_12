"""
Taran Soden (Team 12 CV)
SEED Lab - Fall 2021
Camera Matrix Detection (USING CHESSBOARD)

Requires x images in .jpg format in the same file directory to function

"""

import numpy as np
import cv2 as cv
import glob

# constants and needed variables
CHECKERBOARD = (6,9) # Dimensions
i = 0
images = glob.glob('*.jpg') # Get images from directory

# arrays to store object points and image points from all the images
objpoints = [] # 3D IRL
imgpoints = [] # 2D image plane

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)
prev_img_shape = None

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    # find the chessboard corners
    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)
    
    # if found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        
        corners2 = cv.cornerSubPix(gray, corners,(11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        
        # draw and display corners
        img = cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(0)
        
        h,w = img.shape[:2]

        ret,mtx,dist,rvecs,tvecs = cv.calibrateCamera(objpoints,imgpoints,gray.shape[::-1],None,None)
        
        i = i+1
        print("\n---- Image %s ----" %i)
        print("\nCamera matrix : \n")
        print(mtx)
        print("\nDist : \n")
        print(dist)
        
cv.destroyAllWindows()
