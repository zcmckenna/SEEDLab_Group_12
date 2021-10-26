"""
Taran Soden (Team 12 CV)
SEED Lab - Fall 2021
Remove Camera Distortion and Camera Matrix Setup
"""
import numpy as np
import cv2 as cv

# ---- Properties ----
# camera matrix and distortion matrix for camera held by CV for testing (determined with averages)
mtx  = np.array([[ 613.42869186, 0,            324.11185720],
                 [ 0,            611.99367189, 233.78141328],
                 [ 0,            0,            1           ]])

dist = np.array([ 0.10044397, -0.26589755, -0.00337026, -0.00480078, -0.66517406 ])


# ---- Functions ----
'''
Name:      remove_distortion()
Function:  Input an image, apply the camera matrix adjustment, return the adjusted image
Input:     image object (numpy array), showComparison (bool)
Output:    adjusted image and new width and height
'''
def remove_distortion(image, showComparison):
    
    h,w = image.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h)) # get new optimal camera matrix
    
    dst = cv.undistort(image, mtx, dist, None, newcameramtx) # undistort

    x,y,w,h = roi           # define nww region of interest (ROI)
    dst = dst[y:y+h, x:x+w] # crop the adjusted image
    
    # original image and new image can be shown if requested
    if showComparison is None:
        showComparison = False
    if showComparison:
        cv.imshow('original', image)
        cv.imshow('calibration', dst)
        cv.waitKey(0)
        cv.destroyAllWindows()
    
    return dst, w, h


'''
Name:      get_cam_matrix()
Function:  Return the camera matrix and distortion matrix
Input:     None
Output:    None
'''
def get_cam_matrix():
    return mtx, dist


'''
Name:      test_cam_matrix()
Function:  Input an image name/path, apply the camera matrix adjustment 
Input:     image name
Output:    None
'''
def test_cam_matrix(imName):
    img = cv.imread('image4.jpg')
    h,w = img.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

    dst = cv.undistort(img, mtx, dist, None, newcameramtx)

    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    
    cv.imshow('orig', img)
    cv.imshow('calibration', dst)
    cv.waitKey(0)
    cv.destroyAllWindows()
    
    