import numpy as np
import cv2
import glob
import depthai as dai
import math
import matplotlib.pyplot as plt

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (8,6)
frameSize = (1280,800)


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 25
objp = objp * size_of_chessboard_squares_mm

def calibrate(imageFiles):

    objPoints = []
    imgPoints = []
    success = 0

    for imgFile in imageFiles:
        img = cv2.imread(imgFile)
        grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # its already gray but oh well

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(grayImg, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            success += 1
            objPoints.append(objp) # same for every image because all the chessboard should be visible
            corners = cv2.cornerSubPix(grayImg, corners, (11,11), (-1,-1), criteria)
            imgPoints.append(corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, chessboardSize, corners, ret)
            cv2.imshow('img', img)
            cv2.waitKey(1000)

    print("processing calibration...")

    ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, frameSize, None, None)
    print("return value: ", ret)
    height, width, channelsL = img.shape
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (width, height), 1, (width, height))

    print("Camera matrix:")
    print(newCameraMatrix)
    print("Distortion coefficients:")
    print(distCoeffs)
    print("pictures used:", success)

    cv2.destroyAllWindows()

    return cameraMatrix, newCameraMatrix, distCoeffs

imagesLeft = sorted(glob.glob('images/left/*.png'))
cameraMatrixL, newCameraMatrixL, distLeft = calibrate(imagesLeft)

imagesRight = sorted(glob.glob('images/right/*.png'))
cameraMatrixR, newCameraMatrixR, distRight = calibrate(imagesRight)

def get_from_device():
    with dai.Device() as dev:
            
        # calibFile = 'calib_184430102173351300.json'
        # with open(calibFile) as f:
        #     jsonCalib = f.read()
        # # calibData = dai.CalibrationHandler.fromJson(jsonCalib)
        calibData = dev.readCalibration()
        leftId = dai.CameraBoardSocket.CAM_B
        rightId = dai.CameraBoardSocket.CAM_C

        print(leftId, rightId)

        print(calibData.getBaselineDistance())

        distLeft = np.array(calibData.getDistortionCoefficients(leftId))
        distRight = np.array(calibData.getDistortionCoefficients(rightId))
        cameraMatrixL = np.array(calibData.getCameraIntrinsics(leftId, 1280, 720))
        cameraMatrixR = np.array(calibData.getCameraIntrinsics(rightId, 1280, 720))
        return cameraMatrixL, cameraMatrixR, distLeft, distRight

def test(imageFiles, cameraMatrix, newCameraMatrix, distCoeffs, title):
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (1280, 720), 1, (1280, 720))
    map1, map2 = cv2.initUndistortRectifyMap(cameraMatrix, distCoeffs, None, newCameraMatrix, frameSize, cv2.CV_16SC2)

    for imgFile in imageFiles:
        img = cv2.imread(imgFile)
        img2 = cv2.remap(img, map1, map2, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(img2, chessboardSize, None)
        # corners = cv2.cornerSubPix(img2, corners, (11,11), (-1,-1), criteria)

        # Draw and display the corners
        cv2.drawChessboardCorners(img2, chessboardSize, corners, ret)
        cv2.imshow(title, img2)
        k = cv2.waitKey(0)

        if k == ord('q'):
            return

# cameraMatrixL, cameraMatrixR, distLeftDev, distRightDev = get_from_device()
# newCameraMatrixL = cameraMatrixL
# newCameraMatrixR = cameraMatrixR

test(imagesLeft, cameraMatrixL, newCameraMatrixL, distLeft, "left")
test(imagesRight, cameraMatrixR, newCameraMatrixR, distRight, "right")

def plot_radial_distortion(distortionCoeff: np.ndarray, ax):

    height = 720
    width = 1280
    focal_length = 800

    if np.size(distortionCoeff) < 5:
        print('missing coefficients')
        return
    
    max_r = math.sqrt(((height / (2*focal_length)) ** 2) + ((width / (2 * focal_length)) ** 2))
    x = np.linspace(0,max_r)
    y = 1 + distortionCoeff[0] * (x ** 2) + distortionCoeff[1] * (x ** 4) + distortionCoeff[4] * (x ** 6)

    if len(distortionCoeff) >= 8:
        y = y / (1 + distortionCoeff[5] * (x ** 2) + distortionCoeff[6] * (x ** 4) + distortionCoeff[7] * (x ** 6))
    
    ax.plot(x, y)

distLeftOld = np.array([0.10848258, -0.43337572,  0.0008588,  -0.00067206 , 0.38446405])
distRightOld = np.array([0.12764481, -0.55572187,  0.00221242, -0.00065469,  0.59227522])

distLeft = np.array([1.12475850e-01, -4.73456873e-01,  4.49832290e-04,  3.36179599e-04, 4.53139814e-01])
distRight = np.array([0.09678987, -0.37351912,  0.00139157, -0.00054162,  0.3380022])

fig, ax = plt.subplots()

plot_radial_distortion(distLeft, ax)
plot_radial_distortion(distRight, ax)
# plot_radial_distortion(distLeftOld, ax)
# plot_radial_distortion(distRightOld, ax)
# plot_radial_distortion(distLeftDev, ax)
# plot_radial_distortion(distRightDev, ax)


ax.set(ylim=(0.95, 1.05))
fig.legend(('left','right')) # , 'left old', 'right old', 'left factory', 'right factory'))

plt.show()


# ########## Stereo Vision Calibration #############################################

# flags = 0
# flags |= cv2.CALIB_FIX_INTRINSIC
# # Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
# # Hence intrinsic parameters are the same 

# criteria_stereo= (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# # This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
# retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)




# ########## Stereo Rectification #################################################

# rectifyScale= 1
# rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv2.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0))

# stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv2.CV_16SC2)
# stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv2.CV_16SC2)

# print("Saving parameters!")
# cv_file = cv2.FileStorage('stereoMap.xml', cv2.FILE_STORAGE_WRITE)

# cv_file.write('stereoMapL_x',stereoMapL[0])
# cv_file.write('stereoMapL_y',stereoMapL[1])
# cv_file.write('stereoMapR_x',stereoMapR[0])
# cv_file.write('stereoMapR_y',stereoMapR[1])

# cv_file.release()