#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import rospkg

if __name__ == '__main__':
    
    # initiate get_intrinsic_parameter node
    rospy.init_node('get_intrinsic_parameter', anonymous=True)
        
    CHECKERBOARD = (4,6)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 


    # Defining the world coordinates for 3D points
    objp = np.zeros((1, 24, 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:4, 0:6].T.reshape(-1, 2)

    # Extract images
    images = []
    
    # get current directory
    rospack = rospkg.RosPack()
    pkg_name = rospack.get_path('lab7')
    
    for i in range(1,8):
        name = pkg_name + '/images/' + str(i) + '.jpg' 
        image = cv2.imread(name)
        images.append(image)
    
    # get corners
    for img in images:
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)            
            imgpoints.append(corners2)

    # get camera intrinsic matrix    
    _, mtx, _, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Camera intrinsic matrix : \n")
    print(mtx)


    rospy.spin()