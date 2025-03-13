import cv2
import os
from natsort import natsorted

def create_video_from_frames(frames_folder, output_video, fps=30, file_extensions=('.png', '.jpg', '.jpeg')):
    """
    Erstellt ein Video aus Frames in einem angegebenen Ordner.
    
    :param frames_folder: Pfad zum Ordner mit Frames
    :param output_video: Name der Ausgabedatei (z. B. 'output.mp4')
    :param fps: Bildrate in Frames pro Sekunde (Default: 30.0)
    :param file_extensions: Tuple mit zulässigen Dateiendungen (Default: ('.png', '.jpg', '.jpeg'))
    """
    # Alle Bilddateien im Ordner sortiert laden
    frame_files = natsorted([f for f in os.listdir(frames_folder) if f.endswith(file_extensions)])
    frame_files = frame_files[:]
    if not frame_files:
        raise ValueError(f"Keine passenden Dateien in {frames_folder} gefunden.")
    
    # Den ersten Frame laden, um die Frame-Größe zu bestimmen
    first_frame = cv2.imread(os.path.join(frames_folder, frame_files[0]))
    if first_frame is None:
        raise ValueError("Die erste Bilddatei konnte nicht geladen werden. Überprüfe den Ordner.")
    
    height, width, _ = first_frame.shape
    
    # VideoWriter initialisieren
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec für MP4
    video_writer = cv2.VideoWriter(output_video, fourcc, fps, (width, height))
    
    # Alle Frames zum Video hinzufügen
    for frame_file in frame_files:
        frame_path = os.path.join(frames_folder, frame_file)
        frame = cv2.imread(frame_path)
        if frame is None:
            print(f"Warnung: {frame_file} konnte nicht geladen werden und wird übersprungen.")
            continue
        video_writer.write(frame)  # Frame hinzufügen
    
    # Writer schließen
    video_writer.release()
    print(f"Video erstellt: {output_video}")

current_dir = os.getcwd()
# Ordner mit den Frames
frames_folder = os.path.join(current_dir, "radar_cam_short_clip")
output_video = "radar_cam_short_with_colors_clip.mp4"
fps = 51.02  # Bildrate in Hz

create_video_from_frames(frames_folder, output_video, fps=fps)