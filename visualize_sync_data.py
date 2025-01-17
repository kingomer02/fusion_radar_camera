import pickle
import matplotlib.pyplot as plt
import numpy as np
import cv2
from center_image_undistorted import undistorted_image_center
from radar_to_camera import *
import os

def visualize_pair(data, frame_path):
    """
    Visualisiert ein synchronisiertes Paar aus Radar- und Kameradaten.
    """
    # Radar-Daten (Dummy-Visualisierung)
    radar_data = data["radar"]["data"]
    
    # Punktwolke dekodieren
    points = np.array(radar_data, dtype=np.float32).reshape(-1, 4)  # x, y, z, intensity
    
    # Punkte in separate Arrays zerlegen
    x_coords = points[:, 0]
    y_coords = points[:, 1]
    z_coords = points[:, 3]
    intensities = points[:, 3]

    # Radar zu Kamera transformieren
    points_base = transform_points_radar_base(points[:, :3])
    base_camera = transform_points_base_camera(points_base)
    camera_points = transform_axes_to_camera(base_camera)

    # Projektion ins Bild
    camera_matrix_center = np.array([[1459.216265, 0.000000, 661.247822],
                           [0.000000, 1456.105734, 514.219839],
                           [0.000000, 0.000000, 1.000000]])

    camera_proj = project_points(camera_points, camera_matrix_center)

    x_coords_cam = camera_proj[:, 0]
    y_coords_cam = camera_proj[:, 1]

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

        # Skaliere den Radius, damit er ins Bild passt (z.B. auf einen max. Wert von 250 Pixel)
        r = min(r, MAX_RANGE)

        # Umwandlung von Polar in kartesische Koordinaten für das Radarbild
        radar_x = int(r * np.cos(theta) + radar_image.shape[1] // 2)
        radar_y = int(-r * np.sin(theta) + radar_image.shape[0] // 2) # im Uhrzeigersinn drehen (-r)

        # Zeichne die Punkte im Radarbild
        intensity = np.clip(intensity, 0, 255)  # Begrenze die Intensität
        cv2.circle(radar_image, (radar_x, radar_y), 2, (0, intensity, 0), -1)  # Grüne Punkte für Radar

    # Die Koordinaten begrenzt auf das Bild
    y_coords_n = y_coords[valid_mask]
    x_coords_n = x_coords[valid_mask]

    # Beziehe die Winkel der Punkte
    theta_n = np.arctan2(y_coords_n, x_coords_n)
    
    ind = 0
    for x, y in zip(x_coords_camn, y_coords_camn):
        theta_i = theta_n[ind]
        theta_i = theta_i + np.pi / 2  # Verschiebe den Winkel um 90° (π/2)
        
        # Wenn der Winkel negativ ist, mache ihn positiv
        if theta_i < 0:
            theta_i += 2 * np.pi

        # Filtert den bereich zum Anzeigen der Punkte
        if theta_i >= 1.2 and  theta_i <= 3.14:
            cv2.circle(image_rgb, (int(x), int(y)), 3, (0, 255, 0), -1)  # Grüne Punkte
        
        ind+=1

    # für Plt (Fortführung):
    # # Kamera-Bild auf zweiter Achse
    # ax_image = axes[1]
    # ax_image.imshow(image_rgb)
    # ax_image.set_title("Kamera Bild")
    # ax_image.axis("off")
    
    # Layout anpassen und anzeigen
    # plt.tight_layout()
    # plt.show()
    
    # Speichere das bild im Ordner
    # loc = r"/mnt/c/Users/moakg/OneDrive/Documents/Bachelor/visualized_radar_and_points_in_camera"
    # plt.savefig(loc + f"/radar_points_in_cam_{count_save_fig}.png")

    # Schließe die Figure, um Speicher zu sparen
    # plt.close()

    
    # Füge Kreise hinzu, die Entfernungen markieren
    for r in [int(NOMINAL_RANGE / 2), NOMINAL_RANGE, MAX_RANGE]:
        cv2.circle(radar_image, (radar_image.shape[1] // 2, radar_image.shape[0] // 2), r, (255, 255, 255), 2)  # Weiße Kreise

        # Füge Text für den Radius hinzu
        cv2.putText(radar_image, f"{r}m", 
                    (radar_image.shape[1] // 2 + r, radar_image.shape[0] // 2 - 5),  # Position des Texts
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
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
    # cv2.imshow("Radar und Kamera Visualisierung", resized_combined_image)

    # Speichere das Bild
    cv2.imwrite(frame_path, resized_combined_image)
   

current_dir = os.getcwd()

file_name = "synchronized_pairs_radar_camera_v1.pkl"
SYNCHRONIZED_FILE = os.path.join(current_dir, file_name)

print(f"Pfad zur Datei: {SYNCHRONIZED_FILE}")

print(os.path.exists(SYNCHRONIZED_FILE))

# Laden der Daten aus der Pickle-Datei
with open(SYNCHRONIZED_FILE, "rb") as f:
    synchronized_pairs = pickle.load(f)
    count_save_fig = 0
    
    for data in synchronized_pairs:
        frame_path = os.path.join(current_dir, "radar_and_camera_with_points", f"radar_cam_points_{count_save_fig}.png")
        visualize_pair(data, frame_path)
        count_save_fig += 1
        
        # Warte für frame_delay Millisekunden (z.B. 30 ms für etwa 33 FPS)
        if cv2.waitKey(30) & 0xFF == ord('q'):  # 'q' drücken, um das Video zu beenden
            break
    cv2.destroyAllWindows()


