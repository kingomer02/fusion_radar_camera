import numpy as np
import cv2

TARGET_HEIGHT = 1080
MAX_RANGE = 977
NOMINAL_RANGE = 500

CAMERA_MATRIX_CENTER = np.array([
        [1700.216265, 0.000000, 661.247822],
        [0.000000, 1700.105734, 514.219839],
        [0.000000, 0.000000, 1.000000]
    ], dtype=np.float32)


D = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000], dtype=np.float32)

HEIGHT, WIDTH = 1024, 1280
K_NEW, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX_CENTER, D, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT))

MAP1, MAP2 = cv2.initUndistortRectifyMap(
    CAMERA_MATRIX_CENTER, D, None, CAMERA_MATRIX_CENTER, (WIDTH, HEIGHT), cv2.CV_16SC2
)
