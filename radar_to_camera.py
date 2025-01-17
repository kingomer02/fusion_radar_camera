import numpy as np

def rotation_matrix(roll, pitch, yaw):
    # Konvertiere Winkel in Bogenmaß
    roll, pitch, yaw = np.radians([roll, pitch, yaw])
    
    # Rotationsmatrizen
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    return R_z @ R_y @ R_x

def transform_points_radar_base(points):
    # Transformation Radar -> Fahrzeug
    translation_vehicle = np.array([0, 0, 1.78])  # XR
    rotation_vehicle = np.array([0, 0, 0])        # XR
    # Translation und Rotation anwenden
    R = rotation_matrix(*rotation_vehicle)
    transformed_points = (R @ points.T).T + translation_vehicle
    return transformed_points

def transform_points_base_camera(points):
    # Transformation Fahrzeug -> Zentrale Kamera
    translation_camera = np.array([-0.94, 0, -0.86])  # XCC
    rotation_camera = np.array([0, -2.5, 0])          # XCC
    # print(points + translation_camera)
    # Translation und Rotation anwenden
    R = rotation_matrix(*rotation_camera)
    transformed_points = (R @ points.T).T + translation_camera
    return transformed_points

def project_points(points, camera_matrix):
    # Normiere Punkte
    points_norm = points / points[:, 2][:, np.newaxis]
    # Projiziere Punkte ins Bild
    projected_points = (camera_matrix @ points_norm.T).T
    return projected_points[:, :2]

def transform_axes_to_camera(camera_points):
    # Rotationsmatrix von Radar zu Kamera (Achsenumstellung)
    R_radar_to_camera = np.array([[0, 1, 0],
                                   [0, 0, -1],
                                   [-1, 0, 0]]
                                )
    # Transformation der Achsen
    camera_points = (R_radar_to_camera @ camera_points.T).T
    return camera_points

# # Test: Radarpunkte (Beispiel)
# radar_points = np.array([[10, 0, -1], [2,3,10], [1, -1, -3]])


# points_vehicle = transform_points_radar_base(radar_points)
# print("base:", points_vehicle)

# points_camera = transform_points_base_camera(points_vehicle)
# print("Kamera Points (from base):", points_camera)

# points_camera_axes = transform_axes_to_camera(points_camera)
# print("Kamera Points (cam axes):", points_camera_axes)

# # Projektion ins Bild
# camera_matrix_center = np.array([[1459.216265, 0.000000, 661.247822],
#                            [0.000000, 1456.105734, 514.219839],
#                            [0.000000, 0.000000, 1.000000]])
# image_points = project_points(points_camera_axes, camera_matrix_center)

# print("Bildkoordinaten:", image_points)
