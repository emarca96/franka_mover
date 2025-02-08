#!/usr/bin/env python3

import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import time 
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


# Costanti per YOLO
YOLO_VERSION = "11s"  # Versione YOLO specificata
SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))  # Directory dinamica dello script Python
MODEL_PATH = os.path.join(SCRIPT_DIR, f"runs_v{YOLO_VERSION}/detect/train/weights/best.pt")  # Percorso relativo al modello
CONFIDENCE_THRESHOLD = 0.5  # Valore minimo di confidenza per rilevare oggetti
UNCERTAINTY_THRESHOLD = 0.2  # Percentuale del 10% della bounding box
TRANSIENT_TIME = 10 # tempo in secondi dopo il quale la coordinata viene inviata a ros se persiste il cambiamento di posizione
DEPTH_VIEW_RF = "camera_depth_optical_frame" # Deve coincidere con il RF dello script in franka_mover

class RealSenseViewer(Node):
    def __init__(self):
        super().__init__('realsense_viewer')

        # Crea il publisher per le coordinate delle mele
        self.apple_coordinates_publisher = self.create_publisher(PoseStamped, 'apple_coordinates_realsense', 10)
        
        # Bridge per convertire da ROS Image a OpenCV
        self.bridge = CvBridge()

        # Sottoscrittori per i topic RGB, Depth e Camera Info
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # Topic RGB
            self.rgb_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',  # Topic Depth
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',  # Topic Camera Info
            self.camera_info_callback,
            10
        )

        # Variabili per memorizzare i dati
        self.rgb_image = None
        self.depth_image = None
        self.intrinsics = None  # Per memorizzare fx, fy, ppx, ppy

        # Flag per indicare se i dati sono stati ricevuti
        self.rgb_received = False
        self.depth_received = False

        self.robot_ready = True  # Stato iniziale del robot
        self.status_subscription = self.create_subscription(
            Bool,
            '/robot_status',
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        self.robot_ready = msg.data
        if self.robot_ready:
            self.get_logger().info("Robot ready, stream of coordinates started")
        else:
            self.get_logger().info("Robot busy, stream of coordinates stopped")


    def rgb_callback(self, msg):
        """Callback per lo stream RGB"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # Converte in immagine OpenCV
            self.rgb_received = True
        except Exception as e:
            self.get_logger().error(f"Errore nella conversione RGB: {e}")

    def depth_callback(self, msg):
        """Callback per lo stream Depth"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')  # Converte in immagine OpenCV
            self.depth_received = True
        except Exception as e:
            self.get_logger().error(f"Errore nella conversione Depth: {e}")

    def camera_info_callback(self, msg):
        """Callback per i parametri intrinseci della fotocamera"""
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'ppx': msg.k[2],
                'ppy': msg.k[5],
            }
    
    def publish_apple_coordinates(self, coordinates):
        """
        Pubblica le coordinate di una mela sul topic apple_coordinates_realsense.
        :param coordinates: Tuple delle coordinate (Xcam, Ycam, Zcam).
        :return: True se il messaggio è stato pubblicato, False altrimenti.
        """
        try:
            # Crea un messaggio PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = DEPTH_VIEW_RF  # Modifica con il tuo frame di riferimento
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            # Imposta posizione e orientamento neutro
            pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = map(float, coordinates)
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0

            #pubblica il messaggio
            self.apple_coordinates_publisher.publish(pose_stamped)
            # self.get_logger().info(f"Coordinate pubblicate: {pose_stamped.pose.position}")
            # print(f"Coordinate pubblicate: X={coordinates[0]:.2f}, Y={coordinates[1]:.2f}, Z={coordinates[2]:.2f}")
            return True
        except Exception as e:
            self.get_logger().error(f"Errore nella pubblicazione: {e}")
            return False


def wait_for_intrinsics(node):
    print("In attesa dei parametri intrinseci...")
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.intrinsics is not None:
            return node.intrinsics


def insert_sorted(coordinates_list, new_coordinate):
    index = 0
    while index < len(coordinates_list) and coordinates_list[index][2] <= new_coordinate[2]:
        index += 1
    coordinates_list.insert(index, new_coordinate)


def detect_objects(frame, model, confidence_threshold):
    results = model.predict(frame, conf=confidence_threshold, verbose=False)
    draw_annotations(frame, results,UNCERTAINTY_THRESHOLD)
    centers_and_confidences = get_bounding_box_centers(results)
    centers = [(u, v) for u, v, _ in centers_and_confidences]
    confidences = [confidence for _, _, confidence in centers_and_confidences]
    return frame, centers, confidences


def draw_annotations(frame, results, uncertainty_threshold):
    """
    Disegna le annotazioni sul frame, inclusi il bounding box, la confidenza e il rettangolo di incertezza.
    :param frame: Immagine RGB su cui disegnare.
    :param results: Risultati del modello YOLO.
    :param uncertainty_threshold: Percentuale della soglia di incertezza (0-1).
    """
    for box in results[0].boxes:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
        confidence = box.conf[0] * 100
        u = int((x_min + x_max) / 2)
        v = int((y_min + y_max) / 2)

        box_color = (255, 0, 0)  # Blu
        text_color = (255, 255, 255)  # Bianco
        center_color = (0, 255, 0)  # Verde
        uncertainty_color = (255,0,255) #viola

        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), box_color, 1)
        cv2.circle(frame, (u, v), 2, center_color, -1)

        label = f"{confidence:.1f}%"
        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
        label_rect_x_min = x_min
        label_rect_y_min = y_min - label_size[1] - 5
        label_rect_x_max = x_min + label_size[0] + 10
        label_rect_y_max = y_min

        if label_rect_y_min < 0:
            label_rect_y_min = 0

        cv2.rectangle(frame, (label_rect_x_min, label_rect_y_min), (label_rect_x_max, label_rect_y_max), box_color, -1)
        cv2.putText(frame, label, (x_min + 5, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

        # Calcola il rettangolo di incertezza
        box_width = x_max - x_min
        box_height = y_max - y_min
        uncertainty_width = int(box_width * uncertainty_threshold)
        uncertainty_height = int(box_height * uncertainty_threshold)

        # Coordinate del rettangolo di incertezza
        uncertainty_x_min = u - uncertainty_width // 2
        uncertainty_y_min = v - uncertainty_height // 2
        uncertainty_x_max = u + uncertainty_width // 2
        uncertainty_y_max = v + uncertainty_height // 2

        # Disegna il rettangolo di incertezza
        cv2.rectangle(frame, (uncertainty_x_min, uncertainty_y_min), (uncertainty_x_max, uncertainty_y_max), uncertainty_color, 1)

def get_bounding_box_centers(results):
    """
    Estrae i centri delle bounding box e le percentuali di confidenza.
    :param results: Risultati del modello YOLO.
    :return: Lista di tuple (centro_x, centro_y, confidenza).
    """
    centers_and_confidences = []
    for box in results[0].boxes:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])
        u = int((x_min + x_max) / 2)
        v = int((y_min + y_max) / 2)
        confidence = box.conf[0] * 100  # Percentuale di confidenza
        centers_and_confidences.append((u, v, confidence))
    return centers_and_confidences

def pixel_to_3d(u, v, z, fx, fy, ppx, ppy):
    Xcam = ((u - ppx) * z / fx)/1000
    Ycam = ((v - ppy) * z / fy)/1000
    Zcam = (z)/1000
    return Xcam, Ycam, Zcam


def print_coordinates(coordinates_list, confidences, first_sended):
    """
    Stampa le coordinate delle mele con la percentuale di confidenza.
    Aggiunge '(SENDED)' alla prima riga se le coordinate sono state inviate a ROS.
    :param coordinates_list: Lista di tuple (Xcam, Ycam, Zcam).
    :param confidences: Lista delle percentuali di confidenza corrispondenti.
    :param first_sended: Booleano, True se la prima coordinata è stata inviata.
    """
    print(f"Mele riconosciute: {len(coordinates_list)}")
    for i, ((Xcam, Ycam, Zcam), confidence) in enumerate(zip(coordinates_list, confidences), start=1):
        sended_marker = " (SENDED)" if i == 1 and first_sended else ""
        print(f"{i}) X: {Xcam:.4f} Y: {Ycam:.4f} Z: {Zcam:.4f} {confidence:.4f}%{sended_marker}")


def calculate_uncertainty_box(x_min, y_min, x_max, y_max, percentage):
    """
    Calcola un riquadro di incertezza attorno al centro della bounding box.
    :param x_min: Coordinata x_min della bounding box.
    :param y_min: Coordinata y_min della bounding box.
    :param x_max: Coordinata x_max della bounding box.
    :param y_max: Coordinata y_max della bounding box.
    :param percentage: Percentuale rispetto alle dimensioni della bounding box.
    :return: Coordinate del riquadro di incertezza (x1, y1, x2, y2).
    """
    width = x_max - x_min
    height = y_max - y_min
    center_x = (x_min + x_max) // 2
    center_y = (y_min + y_max) // 2

    # Dimensioni del riquadro di incertezza
    delta_x = int(width * percentage / 2)
    delta_y = int(height * percentage / 2)

    # Calcolo delle coordinate
    x1 = center_x - delta_x
    y1 = center_y - delta_y
    x2 = center_x + delta_x
    y2 = center_y + delta_y

    return x1, y1, x2, y2









def main(args=None):
    rclpy.init(args=args)
    realsense_viewer = RealSenseViewer()

    os.environ['YOLO_LOG_LEVEL'] = 'silent'
    print("Caricamento del modello YOLO...")
    model = YOLO(MODEL_PATH)
    print("Yolo apple-detection caricato correttamente")

    intrinsics = wait_for_intrinsics(realsense_viewer)
    fx, fy, ppx, ppy = intrinsics['fx'], intrinsics['fy'], intrinsics['ppx'], intrinsics['ppy']
    print("Parametri intrinseci ricevuti.")

    # Flag per mostrare i messaggi solo una volta
    rgb_waiting_message_shown = False
    rgb_received_message_shown = False
    depth_waiting_message_shown = False
    depth_received_message_shown = False

    ros_sended = None  # Variabile per tracciare le coordinate già inviate
    variation_timestamp = None # Timestamp per tracciare una variazione significativa
    try:
        while rclpy.ok():
            rclpy.spin_once(realsense_viewer)
	        
            if not realsense_viewer.robot_ready:
                continue  # Salta questa iterazione del ciclo se il robot non è pronto

            first_sended = False
	    
            # Gestione dello stato RGB
            if realsense_viewer.rgb_image is None:
                if not rgb_waiting_message_shown:
                    print("In attesa dello stream RGB...")
                    rgb_waiting_message_shown = True
            else:
                if not rgb_received_message_shown:
                    print(f"Stream RGB ricevuto: {realsense_viewer.rgb_image.shape}")
                    rgb_received_message_shown = True

            # Gestione dello stato Depth
            if realsense_viewer.depth_image is None:
                if not depth_waiting_message_shown:
                    print("In attesa dello stream Depth Allineato...")
                    depth_waiting_message_shown = True
            else:
                if not depth_received_message_shown:
                    print(f"Stream Depth Allineato ricevuto: {realsense_viewer.depth_image.shape}")
                    depth_received_message_shown = True

            # Procedi solo se entrambi gli stream sono disponibili
            if realsense_viewer.rgb_image is not None and realsense_viewer.depth_image is not None:
                apple_coordinates_realsense = []
                detected_frame, centers, confidences = detect_objects(
                    realsense_viewer.rgb_image.copy(), model, CONFIDENCE_THRESHOLD
                )
                
                for u, v in centers:
                    depth_value = realsense_viewer.depth_image[v, u]
                    Xcam, Ycam, Zcam = pixel_to_3d(u, v, depth_value, fx, fy, ppx, ppy)
                    insert_sorted(apple_coordinates_realsense, (Xcam, Ycam, Zcam))

                if apple_coordinates_realsense:  # Se ci sono coordinate rilevate
                    first_coordinate = apple_coordinates_realsense[0]  # Prendi la prima mela
                    first_sended = False #flag per tracciare se la mela è stata inviata

                    # Controlla se inviare o meno le coordinate: le invia se o x, o y, o z si discostano dalla soglia
                    # e non le invia mai quando la Z = 0, perchè e un dato outlier di transitorio.ù
                    
                    if ros_sended is None or (
                          first_coordinate[2] != 0 and (
                          abs(float(first_coordinate[0] - ros_sended[0])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[0]) or \
                          abs(float(first_coordinate[1] - ros_sended[1])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[1]) or \
                          abs(float(first_coordinate[2] - ros_sended[2])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[2]) 
                          )  
                        ):
                        
                            if variation_timestamp is None:
                                # Inizializza il timestamp della variazione
                                variation_timestamp = time.time()
                            else:
                                # Controlla se sono passati tot secondi
                                if time.time() - variation_timestamp >= TRANSIENT_TIME:
                                    # Conferma se la variazione persiste
                                    if  ros_sended is None or ( first_coordinate[2] != 0 and (
                                        abs(float(first_coordinate[0] - ros_sended[0])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[0]) or \
                                        abs(float(first_coordinate[1] - ros_sended[1])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[1]) or \
                                        abs(float(first_coordinate[2] - ros_sended[2])) > UNCERTAINTY_THRESHOLD * abs(ros_sended[2]) 
                                        )): 
                                        
                                        # Pubblica la coordinata
                                        first_sended = realsense_viewer.publish_apple_coordinates(first_coordinate)
                                        if first_sended:
                                            ros_sended =  tuple(map(float, first_coordinate))
                                            variation_timestamp = None  # Reset del timestamp
                else:
                    # Reset del timestamp se non c'è variazione
                    variation_timestamp = None
                            

                print_coordinates(apple_coordinates_realsense,confidences, first_sended)
                cv2.imshow('RGB Stream with YOLO', detected_frame)
            
            # Visualizzo Depth Map
            #if realsense_viewer.depth_image is not None:
                #depth_colormap = cv2.applyColorMap(
                    #cv2.convertScaleAbs(realsense_viewer.depth_image, alpha=0.03),
                    #cv2.COLORMAP_JET,
                #)
                #cv2.imshow('Depth Stream', depth_colormap)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        realsense_viewer.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
