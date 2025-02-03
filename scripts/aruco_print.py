import cv2
import numpy as np
import cv2.aruco as aruco

def generate_aruco_marker(dictionary_type=aruco.DICT_7X7_250, marker_id=0, marker_size=700, output_filename="aruco_marker7x7.png"):
    """
    Genera un marker ArUco e lo salva come immagine.
    
    :param dictionary_type: Tipo di dizionario ArUco (default: DICT_6X6_250)
    :param marker_id: ID del marker da generare
    :param marker_size: Dimensione del marker in pixel
    :param output_filename: Nome del file immagine di output
    """
    aruco_dict = aruco.getPredefinedDictionary(dictionary_type)
    
    # Genera il marker
    marker_image = aruco_dict.generateImageMarker(marker_id, marker_size)
    
    # Salva l'immagine
    cv2.imwrite(output_filename, marker_image)
    print(f"Marker ArUco ID {marker_id} salvato come {output_filename}")

if __name__ == "__main__":
    generate_aruco_marker()
