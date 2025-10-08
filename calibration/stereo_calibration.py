import numpy as np
import cv2
import glob


################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (8,6)
frameSize = (1280,720)


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 25
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

imagesLeft = sorted(glob.glob('images_old/left/*.png'))
imagesRight = sorted(glob.glob('images_old/right/*.png'))

for imgLeft, imgRight in zip(imagesLeft, imagesRight):

    imgL = cv2.imread(imgLeft, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(imgRight, cv2.IMREAD_GRAYSCALE)

    # Find the chess board corners
    retL, cornersL = cv2.findChessboardCorners(imgL, chessboardSize, None)
    retR, cornersR = cv2.findChessboardCorners(imgR, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if retL and retR == True:

        objpoints.append(objp)

        cornersL = cv2.cornerSubPix(imgL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR = cv2.cornerSubPix(imgR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        cv2.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv2.imshow('img left', imgL)
        cv2.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
        cv2.imshow('img right', imgR)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# use distortion parameters and camera matrix from factory calibration
cameraMatrixL = np.array([
    [797.22247314,   0.,       636.40203857],
    [0.,         797.22247314, 356.68164062],
    [0.,         0.,           1.        ]])

cameraMatrixR = np.array([
    [801.16522217,   0.,         653.42108154],
    [  0.,         801.16522217, 360.30673218],
    [  0.,           0.,           1.        ]
])

# # camera matrices (for 1280 x 800)
# cameraMatrixL = np.array([
#     [797.2224731445312, 0.0, 636.4020385742188], 
#     [0.0, 797.2224731445312, 396.681640625], 
#     [0.0, 0.0, 1.0]
# ])
# cameraMatrixR = np.array([
#     [801.1652221679688, 0.0, 653.4210815429688],
#     [0.0, 801.1652221679688, 400.3067321777344],
#     [0.0, 0.0, 1.0]
# ])


distL = np.array([
    4.311811923980713,
    10.515290260314941,
    0.0005457720835693181,
    0.0019910847768187523,
    -25.130577087402344,
    4.021228790283203,
    11.57146167755127,
    -26.03287124633789
    ])

distR = np.array([
    -7.056822299957275,
    54.57760238647461,
    0.001735908561386168,
    0.00025019279564730823,
    -24.580093383789062,
    -7.1168622970581055,
    54.46717071533203,
    -23.734722137451172
])

# retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, frameSize, None, None)
# heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roi_L = cv2.getOptimalNewCameraMatrix(cameraMatrixL, distL, frameSize, 0, frameSize)

# retR, cameraMatrixR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, frameSize, None, None)
# heightR, widthR, channelsR = imgR.shape
newCameraMatrixR, roi_R = cv2.getOptimalNewCameraMatrix(cameraMatrixR, distR, frameSize, 0, frameSize)

########## Stereo Vision Calibration #############################################

flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC
flags |= cv2.CALIB_RATIONAL_MODEL
flags |= cv2.CALIB_USE_EXTRINSIC_GUESS
# Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# Hence intrinsic parameters are the same 

# criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
rot = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
])

trans = np.array([-75.0, 0.0, 0.0])

# This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix, pve = cv2.stereoCalibrateExtended(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, imgL.shape[::-1], R=rot, T=trans, flags=flags)

print("return stereo: ", retStereo)
print("newCameraMatrixL", newCameraMatrixL)
print("distL:", distL)
print("NewCameraMatrixR", newCameraMatrixR)
print("distR:", distR)
print("rot:", rot)
print("trans:", trans)
print("essential matrix:", essentialMatrix)
print("fundemantel matrix: ", fundamentalMatrix)
# print("per view errors:", pve)

########## Stereo Rectification #################################################

rectifyScale= 0
rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, imgL.shape[::-1], rot, trans, alpha=rectifyScale)

print("rectL: ", rectL)
print("rectR:", rectR)
print("projMatrixL:", projMatrixL)
print("projMatrixR:", projMatrixR)
print("Q", Q)

stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, imgL.shape[::-1], cv2.CV_16SC2)
stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, imgR.shape[::-1], cv2.CV_16SC2)

print("Saving parameters!")
cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

cv_file.write('stereoMapL_x',stereoMapL[0])
cv_file.write('stereoMapL_y',stereoMapL[1])
cv_file.write('stereoMapR_x',stereoMapR[0])
cv_file.write('stereoMapR_y',stereoMapR[1])
cv_file.write('q_matrix', Q)

cv_file.release()