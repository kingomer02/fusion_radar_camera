import cv2
import numpy as np
from src.config import CAMERA_MATRIX_CENTER, D, HEIGHT, WIDTH, MAP1, MAP2
# import rosbag
# import os

# current_dir = os.getcwd()

def undistorted_image_center(image_buf):
    """
    Nimmt einen Bildpuffer (z.B. von cv2.imdecode) und gibt das entzerrte Bild zurück.
    """
    # Decodieren des Bildes (falls nötig)
    image = cv2.imdecode(image_buf, cv2.IMREAD_COLOR)
    
    # Verwende die vorab berechneten Maps, um das Bild zu entzerren
    undistorted_image = cv2.remap(image, MAP1, MAP2, interpolation=cv2.INTER_LINEAR)
    
    return undistorted_image