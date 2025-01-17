import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import matplotlib.pyplot as plt
import base64
import numpy as np
import os
# import pickle

# Pfad zur Bag-Datei und Topic-Name
BAG_FILE = r"/mnt/c/Users/moakg/OneDrive/Documents/Bachelor/philos_2020_10_22_car_carrier_far/section.bag"
IMAGE_TOPIC = r"/center_camera/image_color/compressed"
RADAR_TOPIC = r"/broadband_radar/channel_0/pointcloud"

# radar_data = np.zeros(18407, dtype=[
#     ("timestamp", np.float64),
#     ("frame_id", "U50"),
#     ("points", object)  # Punkte als Objekt, da sie Arrays mit variabler Größe sind
# ])

# camera_data = []

# # ROS-Bag öffnen und Nachrichten durchlaufen
# bag = rosbag.Bag(BAG_FILE)

# i_radar, i_camera = 0, 0
# for topic, msg, t in bag.read_messages(topics=[RADAR_TOPIC, IMAGE_TOPIC]):
#     timestamp = t.to_sec()  # Zeitstempel in Sekunden
    
    # if topic == RADAR_TOPIC and i_radar <= 18407:
    #     frame_id = msg.header.frame_id
    #     points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
    #     # radar_timestamps.append(timestamp)
    #     # radar_messages.append({"frame_id":frame_id, "data": points})
    #     points = np.array([(p[0], p[1], p[2], p[3]) for p in points], dtype=np.float32)
    #     radar_data[i_radar] = (timestamp, frame_id, points)
    #     i_radar += 1

    # if topic == IMAGE_TOPIC and i_camera <= 4326:
    #     camera_data.append({
    #         "timestamp": timestamp,
    #         "frame_id": msg.header.frame_id,
    #         "image_data": base64.b64encode(msg.data).decode("utf-8")
    #     })
    #     i_camera+=1


# # Extrahiere die Daten
# timestamps = [entry["timestamp"] for entry in camera_data]
# frame_ids = [entry["frame_id"] for entry in camera_data]
# image_data = [base64.b64decode(entry["image_data"]) for entry in camera_data]

# # Konvertiere in ein numpy-Array (heterogen, dtype=object)
# data_array = np.array(list(zip(timestamps, frame_ids, image_data)), dtype=object)
# np.save('radar_data.npy', radar_data)
# np.save('camera_data.npy', np.array(data_array))

# radar_data = np.load('radar_data.npy', allow_pickle=True)
# camera_data = np.load('camera_data.npy', allow_pickle=True)

# camera_data = camera_data[np.argsort(camera_data[:, 0])] 


# Synchronisation der Daten
def synchronize_data(radar_data, camera_data, max_diff=0.07):
    """Synchronisiert Radar- und Kamera-Daten."""
     # Sicherstellen, dass die Kamera-Daten sortiert sind
    pairs = []
    camera_idx = 0
    camera_len = len(camera_data)

    for radar_entry in radar_data:
        radar_time = radar_entry["timestamp"]

        # Suche den nächsten Kamera-Zeitstempel
        while camera_idx < camera_len - 1 and \
              abs(camera_data[camera_idx + 1][0] - radar_time) < abs(camera_data[camera_idx][0] - radar_time):
            camera_idx += 1

        closest_camera_entry = camera_data[camera_idx]

        # Prüfe, ob der Zeitunterschied innerhalb der Toleranz liegt
        if abs(radar_time - closest_camera_entry[0]) <= max_diff:
            pairs.append({
                'radar': {
                    "timestamp": radar_time,
                    "frame_id": radar_entry["frame_id"],
                    "data": radar_entry["points"]
                },

                'center_camera': {
                    "timestamp": closest_camera_entry[0],
                    "frame_id": closest_camera_entry[1],
                    "image_data": closest_camera_entry[2]
                }
            })
    
    return pairs

# Synchronisierte Paare finden
# synchronized_pairs = synchronize_data(radar_data, camera_data)

# # Ergebnisse ausgeben
# print(f"Gefundene Paare: {len(synchronized_pairs)}")

# # Speichern der synchronisierten Paare
# current_dir = os.getcwd()
# save_file_name = 'synchronized_pairs_radar_camera_v1.pkl'

# output_path = os.path.join(current_dir, save_file_name)
# with open(output_path, "wb") as f:
#     pickle.dump(synchronized_pairs, f)