import cv2
import numpy as np
# import rosbag
# import os

# current_dir = os.getcwd()

# # Pfad zur Bag-Datei und Topic-Name
# BAG_FILE = os.path.join(current_dir, r"philos_2020_10_22_car_carrier_far/section.bag")
# RADAR_TOPIC = r"/center_camera/image_color/compressed"

# # Kalibrierungsparameter aus deiner Datei

# camera_matrix = np.array([
#     [1459.216265, 0.000000, 661.247822],
#     [0.000000, 1456.105734, 514.219839],
#     [0.000000, 0.000000, 1.000000]
# ])

# distortion_coefficients = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000])

# bag = rosbag.Bag(BAG_FILE)
# for topic, msg, t in bag.read_messages(topics=[RADAR_TOPIC]):
#     compressed_data = np.frombuffer(bytearray(msg.data), dtype=np.uint8)
#     image = cv2.imdecode(compressed_data, cv2.IMREAD_COLOR)

#     # Entzerrung durchführen
#     h, w = image.shape[:2]
#     new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))

#     # Entzerren des Bildes
#     undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)

#     # Optional: Zuschneiden
#     x, y, w, h = roi
#     undistorted_image = undistorted_image[y:y+h, x:x+w]

#     # Bilder anzeigen
#     cv2.imshow("Original Image", image)
#     cv2.imshow("Undistorted Image", undistorted_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     break

def undistorted_image_center(image):
    # Kalibrierungsparameter aus deiner Datei

    camera_matrix = np.array([
        [1459.216265, 0.000000, 661.247822],
        [0.000000, 1456.105734, 514.219839],
        [0.000000, 0.000000, 1.000000]
    ])

    distortion_coefficients = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000])

    # Image Verarbeitung
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    # Entzerrung durchführen
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))

    # Entzerren des Bildes
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)

    # Optional: Zuschneiden
    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y+h, x:x+w]

    return image
