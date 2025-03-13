import pickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
from src.center_image_undistorted import undistorted_image_center
from radar_to_camera_2 import *
from src.radar_to_cam_trans import *
import os
import sys

start_theta = None
theta_indexes = []
next_index = False
first_round_passed = False
i = 0
# zero_arr = np.zeros(977)
# indexes_arr = np.load("theta_indexes.npy")

# radar_data_agg = {index: zero_arr for index in indexes_arr}
counter = 0

def point_color(r):
    """
    0-200: green
    200-400: yellow
    400-600: orange
    600-800: red
    """

    if r <= 200:
        return (0, 255, 0)
    elif r <= 400:
        return (0, 255, 255)
    elif r <= 600:
        return (0, 165, 255)
    else:
        return (0, 0, 255)

def quaternion_to_rotation_matrix(q):
    """
    Konvertiert ein Quaternion in eine Rotationsmatrix.
    """
    
    x, y, z, w = q[0:4]
    # x, y, z, w = -x, -y, -z, w
    # Rotationsmatrix berechnen
    R = np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])
    return R

def correct_with_imu(points, rotation_matrix): # world to imu
    points_matrix = np.array(points)

    transformed_points = points_matrix @ rotation_matrix.T

    return transformed_points

def visualize_pair(data, frame_path):

    """
    Visualisiert ein synchronisiertes Paar aus Radar- und Kameradaten.
    """

    # globale variablen 
    global start_theta
    global theta_indexes
    global next_index
    global first_round_passed
    global i
    global counter


    # Radar-Daten (Dummy-Visualisierung)
    radar_data = data["radar"]["data"]
    radar_timestamp = data['radar']['timestamp']
    # print(f"Radar ID: {radar_timestamp}")
    
    # Punktwolke dekodieren
    points = np.array(radar_data, dtype=np.float32).reshape(-1, 4)  # x, y, z, intensity
    
    # Punkte in separate Arrays zerlegen
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    z_coords = points[:, 2]

    intensities = points[:, 3]

    # try:
    #     # Imu Rotation Anpassung
    #     imu_rotation = data["tf"]["imu_rotation"]
    #     imu_rot_matrix = quaternion_to_rotation_matrix(imu_rotation)
    #     corrected_imu_points = correct_with_imu(points[:, :3], imu_rot_matrix)

    #     # Radar zu Kamera transformieren
    #     points_base = transform_points_radar_base(corrected_imu_points)
    # except:
    # points_base = transform_points_radar_base(points[:, :3])

    # base_camera = transform_points_base_camera(points_base)
    # camera_points = transform_axes_to_camera(base_camera)

    # Projektion ins Bild
    camera_matrix_center = np.array([[980.216265, 0.000000, 661.247822],
                           [0.000000, 980.105734, 514.219839],
                           [0.000000, 0.000000, 1.000000]])

    # camera_matrix_center = np.array([[1800, 0.000000, 661.247822],
    #                        [0.000000, 1800, 514.219839],
    #                        [0.000000, 0.000000, 1.000000]])
    

    # camera_proj = project_points(camera_points, camera_matrix_center)
    # camera_proj = transform_radar_to_image(points[:, :3])
    try:
        camera_proj = run_transform_with_imu(points[:, :3], data["tf"]["imu_rotation"], camera_matrix_center)
    except:
        camera_proj = run_transform(points[:, :3], camera_matrix_center)
    # print(camera_proj)
    x_coords_cam = camera_proj[0]
    y_coords_cam = camera_proj[1]
    # print(f"Anzahl der Punkte: {len(x_coords_cam)}")
    # stack_camera_proj = np.column_stack((x_coords_cam, y_coords_cam))
    # reshaped_stack = stack_camera_proj.reshape(-1, 1, 2)
    # undistort_camera_proj = cv2.undistortPoints(reshaped_stack, camera_matrix_center, distortion_coefficients, P=camera_matrix_center)
    # reshaped_undistort_camera_proj = undistort_camera_proj.reshape(-1, 2)

    # x_coords_cam = reshaped_undistort_camera_proj[:, 0]
    # y_coords_cam = reshaped_undistort_camera_proj[:, 1]

    # Kamera-Bild
    image_data = data["center_camera"]["image_data"]
    image_array = np.frombuffer(image_data, dtype=np.uint8)

    image = undistorted_image_center(image_array)

    # Konvertiere von BGR nach RGB
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_height, image_width, _ = image_rgb.shape

    valid_mask = (x_coords_cam >= 0) & (x_coords_cam < image_rgb.shape[1]) & \
             (y_coords_cam >= 0) & (y_coords_cam < image_rgb.shape[0])

    # Beziehe nur die transformierten Radarpunkte welche im Kamerabild liegen
    x_coords_camn = x_coords_cam[valid_mask]
    y_coords_camn = y_coords_cam[valid_mask]
    
    
    # Alternative zu cv2 mit plt:

    # # Erstelle eine Matplotlib-Figure und Achsen
    # fig, axes = plt.subplots(1, 2, figsize=(18, 9))
    # manager = plt.get_current_fig_manager()
    # manager.full_screen_toggle()

    # # Radar-Daten auf erster Achse (Polarkoordinaten)
    # ax_radar = plt.subplot(121, projection='polar')  # explizit die Polarprojektion für den ersten Plot verwenden
    # r = np.sqrt(x_coords**2 + y_coords**2)  # Radius
    # theta = np.arctan2(y_coords, x_coords)  # Winkel
    # scatter = ax_radar.scatter(theta, r, c=intensities, cmap='viridis', s=1)
    # fig.colorbar(scatter, ax=ax_radar, label="Intensität")
    # ax_radar.set_title("Radar-Punktwolke (Polar)")
    # ax_radar.set_theta_zero_location('N')  # Setze Norden als 0°
    # ax_radar.set_rlim(0, 977)  # Radiale Grenzen von 0 bis 977
    

    # Erstelle ein leeres Bild für das Radar 
    target_height = 2048 # Höhe des Bildes
    radar_image = np.zeros((target_height, target_height, 3), dtype=np.uint8)
    
    # Maximale und Nominale Range für den Radar
    MAX_RANGE = 977
    NOMINAL_RANGE = 500
    

    # Radar-Daten als Punkte auf das Radar-Bild zeichnen
    for x, y, intensity in zip(x_coords, y_coords, intensities):
        # Berechne den Radius (Abstand vom Ursprung) und den Winkel
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)

        # Korrigiere den Winkel, damit 0° oben ist
        theta = theta + np.pi / 2
        
        # Wenn der Winkel negativ ist, mache ihn positiv
        if theta < 0:
            theta += 2 * np.pi

        theta = round(theta, 6)
        # print(theta)

        # # Code for saving the radians
        # if start_theta == None and not (theta == 1.570796 or theta == 4.712389):
        #     start_theta = theta
        #     theta_indexes.append(start_theta)

        # if next_index and theta == start_theta:
        #     i += 1

        # if not (theta == 1.570796 or theta == 4.712389) and theta not in theta_indexes:
        #     theta_indexes.append(theta)
        
        # if theta == 1.570796 or theta == 4.712389:
        #     next_index = True
        # else:
        #     next_index = False
        
        # if i == 4:
        #     theta_indexes = np.array(theta_indexes)
        #     np.save('theta_indexes_1.npy', theta_indexes)
        #     sys.exit()

        # Skaliere den Radius, damit er ins Bild passt (z.B. auf einen max. Wert von 250 Pixel)
        r = min(r, MAX_RANGE)

        # Umwandlung von Polar in kartesische Koordinaten für das Radarbild
        radar_x = int(r * np.cos(theta) + radar_image.shape[1] // 2)
        radar_y = int(-r * np.sin(theta) + radar_image.shape[0] // 2) # im Uhrzeigersinn drehen (-r)

        # Zeichne die Punkte im Radarbild
        # intensity = np.clip(intensity, 0, 255)  # Begrenze die Intensität
        # cv2.circle(radar_image, (radar_x, radar_y), 2, (0, intensity, 0), -1)  # Grüne Punkte für Radar

        radar_image[radar_y, radar_x] = [0, intensity, 0] # Grüne Punkte für Radar

    # Die Koordinaten begrenzt auf das Bild
    y_coords_n = y_coords[valid_mask]
    x_coords_n = x_coords[valid_mask]

    # Beziehe die Winkel der Punkte
    theta_n = np.arctan2(y_coords_n, x_coords_n)

    ind = 0
    for x, y in zip(x_coords_camn, y_coords_camn):
        r = np.sqrt(x_coords_n[ind]**2 + y_coords_n[ind]**2)
        color = point_color(r)
        theta_i = theta_n[ind]
        theta_i = theta_i + np.pi / 2  # Verschiebe den Winkel um 90° (π/2)

        # Wenn der Winkel negativ ist, mache ihn positiv
        if theta_i < 0:
            theta_i += 2 * np.pi

        # Filtert den bereich zum Anzeigen der Punkte
        if theta_i >= 1.1 and theta_i <= 2.04:
            cv2.circle(image_rgb, (int(x), int(y)), 2, color, -1)  # Grüne Punkte

        ind+=1


    
    # Füge Kreise hinzu, die Entfernungen markieren
    center = (radar_image.shape[1] // 2, radar_image.shape[0] // 2)
    for r in [int(NOMINAL_RANGE / 2), NOMINAL_RANGE, MAX_RANGE]:
        cv2.circle(radar_image, center, r, (255, 255, 255), 2)  # Weiße Kreise

        # Füge Text für den Radius hinzu
        cv2.putText(radar_image, f"{r}m", 
                    (radar_image.shape[1] // 2 + r, radar_image.shape[0] // 2 - 5),  # Position des Texts
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    line_length = 977

    theta_cam_left = 1.95 + np.pi # Verschiebe den Winkel um 90° (π/2)
    theta_cam_right = 1.1 + np.pi  # Verschiebe den Winkel um 90° (π/2)
    
    end_point1 = (int(center[0] + line_length * np.cos(theta_cam_left)),
              int(center[1] + line_length * np.sin(theta_cam_left)))
    
    end_point2 = (int(center[0] + line_length * np.cos(theta_cam_right)),
              int(center[1] + line_length * np.sin(theta_cam_right)))
    
    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point1, (255), 2)
    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point2, (255), 2)

    # Zielhöhe definieren
    target_height = radar_image.shape[0]

    # Berechne den Skalierungsfaktor
    scale_factor = target_height / image_rgb.shape[0]
    
    # Neue Breite proportional anpassen
    target_width = int(image_rgb.shape[1] * scale_factor)

    image_resized = cv2.resize(image_rgb, (target_width, target_height))

    # Zeige das Radar-Bild und das Kamera-Bild nebeneinander an
    combined_image = np.hstack((radar_image, image_resized))  # Kombiniere die Bilder horizontal

    # Verkleinere das kombinierte Bild auf eine gewünschte Breite
    resize_width=1920
    height = int(combined_image.shape[0] * (resize_width / combined_image.shape[1]))
    resized_combined_image = cv2.resize(combined_image, (resize_width, height))

    # Zeige das kombinierte Bild im OpenCV-Fenster an
    cv2.imshow("Radar und Kamera Visualisierung", resized_combined_image)

    # Speichere das Bild
    # cv2.imwrite(frame_path, resized_combined_image)


current_dir = os.getcwd()

file_name = "synchronized_pairs_radar_camera_tf.pkl"
SYNCHRONIZED_FILE = os.path.join(current_dir, file_name)

print(f"Pfad zur Datei: {SYNCHRONIZED_FILE}")

print(os.path.exists(SYNCHRONIZED_FILE))

# Laden der Daten aus der Pickle-Datei
with open(SYNCHRONIZED_FILE, "rb") as f:
    synchronized_pairs = pickle.load(f)
    count_save_fig = 0
    
    for data in synchronized_pairs[15000:]:
        frame_path = os.path.join(current_dir, "radar_and_camera_with_points", f"radar_cam_points_{count_save_fig}.png")
        visualize_pair(data, frame_path)
        count_save_fig += 1
        
        # Warte für frame_delay Millisekunden
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

