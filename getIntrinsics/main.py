import numpy as np
import cv2 as cv
import glob
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Calibrates according to images of a chessboard
def get_calibration_matrix(calibrationSquareDimension, width, height):    #sqaure size in meters, width/height: amount of intersections pr. row/column

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((width*height,3), np.float32)
    objp[:,:2] = np.mgrid[0:height,0:width].T.reshape(-1,2) #I think this should be replaced by the actual distance in cm between each checkboard mark.
    objp=objp*calibrationSquareDimension

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob('*.png')

    for filename in images:
        img = cv.imread(filename)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (height,width), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            print("Found on corners on:", filename)
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)
            # Draw and display the corners
            cv.drawChessboardCorners(img, (height,width), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(0)
        else: print("didn't find corners on:", filename)
    cv.destroyAllWindows()
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print(mtx)
    print(dist)

if __name__ == "__main__":
    get_calibration_matrix(0.017, 6, 8)