import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from src.image_processing_funcs import point_color


def generate_radar_points():
    """Erzeugt Testpunkte im Radar-Koordinatensystem."""
    return np.array([
        [280, 0, 0],  # Vorne
        [280, -45, 0],  # Rechts
        [170, -45, 0], # Links
        [3, 0, -2], # Unter Radar
        [3, 0, 2],  # Über Radar
        [2, 2, 2],  # Beliebiger Punkt
    ])

def get_extrinsic_matrix_with_imu(q):
    """Erstellt die Transformationsmatrix von Radar nach Kamera unter Berücksichtigung der IMU-Daten."""
    r = R.from_quat(q)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    # print(pitch)
    # print(yaw)
    # print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
    # Translation in Weltkoordinaten anwenden
    T = np.array([[-0.94], [0.0], [0.92]])

    # Achsentransformation (Radar → Kamera)
    axis_swap = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])  # Achsen umorientieren

    # Rotation basierend auf IMU-Daten (Roll, Pitch, Yaw)
    R_radar_to_cam = R.from_euler('xyz', [pitch-1, 0, -roll], degrees=True).as_matrix()
    R_final = R_radar_to_cam @ axis_swap

    # Erst Translation, dann Rotation
    extrinsic_matrix = np.hstack((R_final, R_final @ T))  # T erst durch R drehen!
    return extrinsic_matrix

def get_extrinsic_matrix():
    """Erstellt die Transformationsmatrix von Radar nach Kamera."""

    T = np.array([[-0.94], [0.0], [0.92]])

    # Achsentransformation (Radar → Kamera)
    axis_swap = np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])  # Achsen umorientieren

    # Rotation basierend auf IMU-Daten (Roll, Pitch, Yaw)
    R_radar_to_cam = R.from_euler('xyz', [0-1, 0, 0], degrees=True).as_matrix()
    R_final = R_radar_to_cam @ axis_swap  

    # Erst Translation, dann Rotation
    extrinsic_matrix = np.hstack((R_final, R_final @ T))  # T erst durch R drehen!

    return extrinsic_matrix

# def project_to_image(points_3d, K):
#     """Projiziert 3D-Punkte in die Bildkoordinaten."""
#     points_h = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))  # Homogene Koordinaten
#     projected = (K @ points_h[:, :3].T).T  # K erwartet 3xN, also nur x, y, z nutzen
#     projected /= projected[:, 2].reshape(-1, 1)  # Perspektivische Division
#     projected_x = projected[:, 0]
#     projected_y = projected[:, 1]
#     distortion_coefficients = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000])
#     undistorted_points = cv2.undistortPoints(np.array([projected_x, projected_y]).reshape(-1,1,2), K, distortion_coefficients)
#     undistorted_points = undistorted_points.reshape(-1, 2)
#     undistorted_points = np.squeeze(undistorted_points)
#     print(undistorted_points)
#     projected_x = undistorted_points[:, 0]
#     projected_y = undistorted_points[:, 1]
#     return projected_x, projected_y   # Nur x, y

def project_to_image(points_3d, K):
    distortion_coefficients = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000])
    # points_h = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))
    points_2d_homogeneous = cv2.projectPoints(points_3d, np.eye(3), np.zeros(3), K, distortion_coefficients)[0]
    points_2d = points_2d_homogeneous.squeeze()
    projected_x = points_2d[:, 0]
    projected_y = points_2d[:, 1]
    return projected_x, projected_y


def visualize(points_2d, img_size=(1280, 1024)):
    """Visualisiert die projizierten Punkte auf einem leeren Bild."""
    img = np.zeros((img_size[1], img_size[0], 3), dtype=np.uint8)
    for x, y in np.stack(points_2d, axis=1):
        x, y = int(x), int(y)
        if 0 <= x < img_size[0] and 0 <= y < img_size[1]:
            cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
    # image distortion
    plt.imshow(img)
    plt.show()

def visualize_with_image(points_2d, img, range):
    """Visualisiert die projizierten Punkte auf einem Kamerabild."""
    print(len(range))
    print(len(points_2d[0]))
    undistorted_img = cv2.undistort(img, K, D)
    i = 0
    for x, y in np.stack(points_2d, axis=1):
        if 0 <= x < undistorted_img.shape[1] and 0 <= y < undistorted_img.shape[0]:
            cv2.circle(undistorted_img, (int(x), int(y)), 1, point_color(range[i]), -1)
        i+=1
        # x, y = int(x), int(y)
        # cv2.circle(img, (x, y), 5, (0, 255, 0), -1)

    cv2.imshow("Image", undistorted_img)
    cv2.waitKey(0)

def run_transform_with_imu(radar_pts, imu, K):
    """Führt die Transformation von Radar- zu Kamerakoordinaten mit imu einbezogen durch und visualisiert die Ergebnisse."""
    extrinsics_with_imu = get_extrinsic_matrix_with_imu(imu)
    # print(extrinsics_with_imu)
    camera_pts_with_imu = (extrinsics_with_imu @ np.hstack((radar_pts, np.ones((radar_pts.shape[0], 1)))).T).T[:, :3]
    # print(camera_pts_with_imu)
    image_pts_with_imu = project_to_image(camera_pts_with_imu, K)
    
    # visualize(image_pts_with_imu)
    return image_pts_with_imu

def run_transform(radar_pts, K):
    """Führt die Transformation von Radar- zu Kamerakoordinaten durch und visualisiert die Ergebnisse."""

    extrinsics = get_extrinsic_matrix()
    # print(extrinsics)
    camera_pts = (extrinsics @ np.hstack((radar_pts, np.ones((radar_pts.shape[0], 1)))).T).T[:, :3]
    # print(camera_pts)
    image_pts = project_to_image(camera_pts, K)
    # print(image_pts)
    # visualize(image_pts)
    return image_pts

def undistort_radar_points(radar_arr, K, D, K_new):

    radar_arr = radar_arr.reshape(-1, 1, 2)
    radar_arr = np.ascontiguousarray(radar_arr, dtype=np.float32)
    undistorted_radar = cv2.undistortPoints(radar_arr, K, D, P=K_new)
    
    undistorted_radar = np.hstack((undistorted_radar[:, 0, :], np.zeros((undistorted_radar.shape[0], 1))))

    return undistorted_radar

# # # Kameramatrix
# K = np.array([
#     [1700.216265, 0.000000, 661.247822],
#     [0.000000, 1700.105734, 514.219839],
#     [0.000000, 0.000000, 1.000000]
# ], dtype=np.float32)

# # D = np.array([-0.020543, 0.346944, 0.0015001, 0.003522, 0.00000], dtype=np.float32)
# # D = np.array([0.01543, 0.186944, 0.001501, 0.003522, 0.00000], dtype=np.float32)
# # D = np.array([0.17059543, 0.086944, 0.0015501, 0.003522, 0.000000], dtype=np.float32)
# # D = np.array([-0.209543, 0.255944, 0.004501, 0.003522, 0.000000], dtype=np.float32)
# # D = np.array([-0.229543, 0.216944, 0.001501, 0.003522, 0.000000], dtype=np.float32)
# D = np.array([-0.259543, 0.186944, 0.001501, 0.003522, 0.000000], dtype=np.float32)

# camera_image = cv2.imread("test_pic_daten/my_saved_frame_48.png")
# h, w = camera_image.shape[:2]
# K_new, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
# # print(K_new)
# # # Testdaten


# import pickle
# with open("test_pic_daten/radar_data.pkl", "rb") as f:
#     radar_points_imu = pickle.load(f)

# radar_points = [data[0] for data in radar_points_imu]

# imu = [data[1] for data in radar_points_imu]

# img_points = []
# ranges = []



# # for i in test:
# #     radar_points.append(np.array(test[i]))
# #     imu.append(np.array([0,0,0,1]))

# # imu_100 = [0,0,0,1]
# # imu.append(np.array(imu_100))



# # radar_points.append(np.array(r_100))

# for i, radar_arr in enumerate(radar_points):
#     if radar_arr.shape[0] == 0:
#         continue
#     # radar_arr = radar_arr[:, :3]
#     radar_arr = radar_arr[:, :2]
#     r = np.sqrt(radar_arr[:, 0]**2 + radar_arr[:, 1]**2)
#     ranges.append(r)

#     undistorted_radar = undistort_radar_points(radar_arr, K, D, K_new)

#     img_points.append(run_transform_with_imu(undistorted_radar, imu[i], K_new))
#     # radar_arr = np.hstack((radar_arr[:, 0, :], np.zeros((radar_arr.shape[0], 1))))
#     # img_points.append(run_transform_with_imu(radar_arr, imu[i], K_new))
#     # print(img_points)

# from itertools import chain
# img_points_x = list(chain(*[i[0] for i in img_points]))
# img_points_y = list(chain(*[i[1] for i in img_points]))


# img_points = np.array([img_points_x, img_points_y])

# ranges = list(chain(*ranges))
# ranges = np.array(ranges)

# visualize_with_image(img_points, camera_image, ranges)


