import numpy as np
import cv2
import scipy.signal as signal
from scipy.ndimage import uniform_filter
from sklearn.cluster import DBSCAN
from scipy import ndimage
import matplotlib.pyplot as plt
import time
from src.config import MAX_RANGE, TARGET_HEIGHT
from src.image_processing_funcs import point_color

def clear_radar_image(radar_image, theta_max, theta_min, length):
    center_x = radar_image.shape[1] // 2
    center_y = radar_image.shape[0] // 2
    
    # Erstelle ein Array für die Radien
    r = np.arange(length)

    if theta_min <= np.float32(0.001534) and theta_max >= np.float32(6.281652):
        unique_thetas = np.arange(0.001534, 0.0782233, 0.001)
        unique_thetas = np.append(unique_thetas, np.arange(6.260176, 6.281652, 0.001))
    else:
        unique_thetas = np.arange(theta_min, theta_max+ 0.0015, 0.001)

    # Erstelle ein 2D-Gitter aus Radien und Winkeln
    R, Theta = np.meshgrid(r, unique_thetas)
    
    # Berechne die x- und y-Koordinaten für alle Punkte im Gitter
    x = (center_x + R * np.cos(Theta)).astype(int)
    y = (center_y - R * np.sin(Theta)).astype(int)
    
    # Filtere die gültigen Indizes, die innerhalb des Bildes liegen
    valid_mask = (
        (x >= 0) & (x < radar_image.shape[1]) &
        (y >= 0) & (y < radar_image.shape[0])
    )
    
    # Setze die gültigen Koordinaten auf Schwarz (0)
    radar_image[y[valid_mask], x[valid_mask]] = (0, 0, 0)


def update_radar_image(radar_image, radar_data_theta):

    all_coords = np.array(radar_data_theta, dtype=np.int32)
    all_coords = all_coords[(all_coords[:, 0] >= 0) & (all_coords[:, 0] < radar_image.shape[1]) & (all_coords[:, 1] >= 0) & (all_coords[:, 1] < radar_image.shape[0])]

    scale = radar_image.shape[0] / (MAX_RANGE * 2)
    center = (radar_image.shape[1] // 2, radar_image.shape[0] // 2)
    distances = np.sqrt((all_coords[:, 0] - center[0])**2 + (all_coords[:, 1] - center[1])**2)
    
    radar_image[all_coords[:, 1], all_coords[:, 0]] = [point_color(int(r/scale)) for r in distances]


def draw_radar_data_agg(radar_image, x_coords_theta, y_coords_theta, MAX_RANGE=MAX_RANGE):
    '''
    Calculate radar points update and clear radar image
    '''
    r = np.sqrt(x_coords_theta**2 + y_coords_theta**2)
    theta = np.arctan2(y_coords_theta, x_coords_theta) + np.pi / 2

    theta[theta < 0] += 2 * np.pi

    theta = np.round(theta, 6).astype(np.float32)

    valid_mask_theta = (theta != 1.570796) & (theta != 4.712389)

    r = r[valid_mask_theta]
    theta = theta[valid_mask_theta]
   
    # intensities = intensities[valid_mask_theta]

    scale = radar_image.shape[0] / (MAX_RANGE * 2)
    r_scaled = r * scale
    radar_x = (r_scaled * np.cos(theta) + radar_image.shape[1] // 2).astype(int)
    radar_y = (-r_scaled * np.sin(theta) + radar_image.shape[0] // 2).astype(int)

    unique_thetas = np.unique(theta)

    theta_min, theta_max = unique_thetas[0], unique_thetas[-1]

    radar_data_theta = []
    clear_radar_image(radar_image, theta_max, theta_min, MAX_RANGE * scale)

    for t, x, y in zip(theta, radar_x, radar_y):
        radar_data_theta.append((int(x), int(y)))
    

    update_radar_image(radar_image, radar_data_theta)


def draw_background_radar(radar_image, ranges, scale=1, TARGET_HEIGHT=TARGET_HEIGHT):
    center = (radar_image.shape[1] // 2, radar_image.shape[0] // 2)


    for r in ranges:
        cv2.circle(radar_image, center, int(r * scale), point_color(r), 1)

        # Füge Text für den Radius hinzu
        cv2.putText(radar_image, f"{r}m", 
            (radar_image.shape[1] // 2 + int(r * scale), radar_image.shape[0] // 2 - 5),  # Position des Texts
            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        
    line_length = TARGET_HEIGHT // 2

    theta_cam_left = 2 + np.pi # Verschiebe den Winkel um 90° (π/2)
    theta_cam_right = 1.15 + np.pi  # Verschiebe den Winkel um 90° (π/2)

    end_point1 = (int(center[0] + line_length * np.cos(theta_cam_left)),
                int(center[1] + line_length * np.sin(theta_cam_left)))

    end_point2 = (int(center[0] + line_length * np.cos(theta_cam_right)),
                int(center[1] + line_length * np.sin(theta_cam_right)))

    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point1, (255, 255, 255), 2)
    cv2.line(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), end_point2, (255, 255, 255), 2)

    for r in [50, 100, 200, 300]:
        cv2.ellipse(radar_image, center, (int(r*scale), int(r*scale)), 0, -np.degrees(1.1), -np.degrees(1.95), point_color(r), 1)
'''
some possible filter functions till now unsuccesful
'''

def tiefpass_filter(points):
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    
    theta = np.arctan2(y_coords, x_coords)

    r = np.sqrt(x_coords**2 + y_coords**2)

    distances_median = signal.medfilt(r, kernel_size=5)
    # filter_r = distances_median

    # filter_r = ndimage.gaussian_filter(r, sigma=1.5)

    # filter_r = cv2.bilateralFilter(distances_median.astype(np.float32), d=5, sigmaColor=75, sigmaSpace=75)
    # filter_r = filter_r.reshape(-1)

    # fs = 51.02
    # cutoff = 10
    # b, a = signal.butter(2, cutoff / (fs / 2), btype='low')  # Butterworth-Tiefpassfilter
    # filter_r = signal.filtfilt(b, a, r)
    filter_r = distances_median

    
    x_filtered = filter_r * np.cos(theta)
    y_filterd = filter_r * np.sin(theta)

    return np.column_stack((x_filtered, y_filterd))


def process_radar_data(points):
    # Extract x and y coordinates
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    
    # Convert to polar coordinates
    theta = np.arctan2(y_coords, x_coords)
    r = np.sqrt(x_coords**2 + y_coords**2)

    # Median Filtering to Remove Noise
    distances_median = signal.medfilt(r, kernel_size=5)

    # Butterworth Low-Pass Filtering (Smooths Data)
    fs = 0.8
    cutoff = 0.15
    b, a = signal.butter(4, cutoff / (fs / 2), btype='low')
    filter_r = signal.filtfilt(b, a, distances_median)

    # Convert Back to Cartesian Coordinates
    x_filtered = filter_r * np.cos(theta)
    y_filterd = filter_r * np.sin(theta)

    # print(len(x_filtered))
    # CFAR - Adaptive Thresholding to Remove Sea Clutter
    guard_cells = 3
    training_cells = 10
    noise_level = uniform_filter(filter_r, size=training_cells)
    detection_threshold = noise_level * 1.05
    detected_targets = np.column_stack((x_filtered, y_filterd))[filter_r > detection_threshold]
    print(len(detected_targets))

    # DBSCAN Clustering - Remove Static Clutter (Land, Buoys)
    clustering = DBSCAN(eps=5, min_samples=10).fit(detected_targets)
    labels = clustering.labels_
    final_filtered_points = detected_targets[labels != -1]  # Remove noise (-1 labels)
    print(final_filtered_points)
    return final_filtered_points

def visualize_cluster(image):
    start_time = time.time()
    # Thresholding anwenden
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    convert_time = time.time() - start_time
    print(f"Konvertierungszeit: {convert_time:.2f} Sekunden")
    _, thresholded = cv2.threshold(image, 100, 255, cv2.THRESH_BINARY)

    # Connected Components für Clustering
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(thresholded)

    clusters = []
    labels_time = time.time() - convert_time
    print(f"Labels-Zeit: {labels_time:.2f} Sekunden")
    # Farbbild erstellen, um die Cluster zu visualisieren
    # output = np.zeros((labels.shape[0], labels.shape[1], 3), dtype=np.uint8)
    for i in range(1, num_labels):  # 0 = Hintergrund, deshalb starten wir bei 1
        x, y = centroids[i]
        w, h, area = stats[i][cv2.CC_STAT_WIDTH], stats[i][cv2.CC_STAT_HEIGHT], stats[i][cv2.CC_STAT_AREA]
        
        # Optionale Filterung kleiner Cluster
        if area < 8:
            continue

        # color = np.random.randint(0, 255, size=(3,), dtype=np.uint8)
        # output[labels == i] = color
        
        # Cluster-Daten speichern
        cluster = {
            'id': i,
            'center_x': x,
            'center_y': y,
            'width': w,
            'height': h,
            'area': area
        }
        clusters.append(cluster)

    # Cluster-Daten ausgeben
    for c in clusters:
        print(f"Cluster {c['id']}: Zentrum=({c['center_x']:.2f}, {c['center_y']:.2f}), "
            f"Breite={c['width']}, Höhe={c['height']}, Fläche={c['area']}")
    process_time = time.time() - convert_time
    print(f"Verarbeitungszeit: {process_time:.2f} Sekunden")
    # Ergebnisse mit cv2 anzeigen
    # cv2.imshow('Originalbild', image)
    # cv2.imshow('Gefundene Cluster', output)
    # show_time = time.time() - process_time
    # print(f"Anzeigezeit: {show_time:.2f} Sekunden")
    # Warten, bis eine Taste gedrückt wird, dann Fenster schließen
