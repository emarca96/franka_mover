import cv2
import numpy as np
from cv2 import aruco

def generate_aruco_marker(dictionary_type=aruco.DICT_6X6_250, marker_id=0, marker_size=700, output_filename="aruco_marker.png"):
    """
    Genera un marker ArUco e lo salva come immagine.
    
    :param dictionary_type: Tipo di dizionario ArUco (default: DICT_6X6_250)
    :param marker_id: ID del marker da generare
    :param marker_size: Dimensione del marker in pixel
    :param output_filename: Nome del file immagine di output
    """
    aruco_dict = aruco.Dictionary_get(dictionary_type)
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)
    
    cv2.imwrite(output_filename, marker_image)
    print(f"Marker ArUco ID {marker_id} salvato come {output_filename}")

if __name__ == "__main__":
    generate_aruco_marker()
