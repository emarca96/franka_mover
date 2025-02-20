#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
import numpy as np
import cv2
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import time
from sensor_msgs.msg import Image, CameraInfo 
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import yaml  # Importa la libreria YAML per salvare i dati
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

ARUCO_SIZE = 0.1
TIME_WINDOW = 20.0  # Tempo in secondi per la media mobile
FRAME_CAMERA = "camera_link"
FRAME_ARUCO = "aruco_marker"
FRAME_HAND = "fr3_hand_tcp"
BASE_FRAME = "fr3_link0"
# Aruco rispetto a fr3_hand_tcp con rotazione X->Y->Z e traslazione(da misurare manualmente)
# Sono date dal supporto montato con faccia dove ce il punto rosso
X_ROT = -90.0  
Y_ROT = -90.0
Z_ROT = 0.0  
X_TRANSLATION = -0.009  # Traslazione -9mm per portarla al pari con filo gommini dei fingers
Y_TRANSLATION = 0.000 # Traslazione per portare il codice aruco sotto
Z_TRANSLATION = 0.069
#### FILE DI CALIBRAZIONE ####
CALIBRATION_FILE = "fr3_camera_calibration.yaml"
config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
calibration_path = os.path.join(config_path, CALIBRATION_FILE)



class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # Inizializzazione TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ======= CONFIGURAZIONE ARUCO ======= #
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.marker_size = ARUCO_SIZE  # Dimensione marker in metri

        # Timer per acquisizione continua
        self.timer = self.create_timer(0.1, self.process_frame)

        self.pose_buffer = []  # Buffer per salvare le ultime trasformazioni

        self.publish_static_fr3_aruco()  # Pubblica la trasformazione statica una volta all'inizio tra aruco e fr3_hand_tcp

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rgb_image = None  # Inizializza l'attributo per evitare errori

        # Sottoscrizione allo stream della telecamera già pubblicato da un altro nodo
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )

        self.valid_acquisition_time = 0.0  # Tempo cumulativo in cui ArUco è stato visto
        self.rgb_image = None  # Inizializza l'attributo

        # Per posa camera_link -> fr3_link0 su file .yaml
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
    def camera_info_callback(self, msg):
        """Riceve i parametri intrinseci della telecamera da ROS2"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array([
                [msg.k[0], 0, msg.k[2]],
                [0, msg.k[4], msg.k[5]],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.get_logger().info("Intrinsec parameters received.")

    def image_callback(self, msg):
        """Riceve il frame della telecamera da ROS2"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f"Error in frame conversion: {e}")

    def publish_static_fr3_aruco(self):
        """Pubblica una trasformazione statica tra fr3_hand_tcp e aruco_marker."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = FRAME_HAND  # fr3_hand_tcp
        static_transform.child_frame_id = FRAME_ARUCO  # aruco_marker

        # Imposta la traslazione fissa
        static_transform.transform.translation.x = X_TRANSLATION
        static_transform.transform.translation.y = Y_TRANSLATION
        static_transform.transform.translation.z = Z_TRANSLATION

        # Calcola la rotazione in quaternioni
        rotation = R.from_euler('XYZ', [X_ROT, Y_ROT, Z_ROT], degrees=True)
        quat = rotation.as_quat()

        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        # Pubblica la trasformazione statica
        static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f"[TF] Static transformation published: {FRAME_HAND} → {FRAME_ARUCO}")

    def process_frame(self):
        if self.rgb_image is None:
            return
        frame = self.rgb_image.copy()
        
        # Convertire in scala di grigi
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Rilevare marker ArUco
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is None:  
            self.valid_acquisition_time = 0.0  # Resetta il tempo se nessun ArUco è visibile
            self.get_logger().warn("No ArUco code seen: valid_acquisition_time is zero!")
        else:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Stimare la posa della telecamera rispetto al marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            for i in range(len(ids)):
                self.publish_tf(tvecs[i], rvecs[i], frame)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                
        # Mostra l'immagine con il marker rilevato
        cv2.imshow("ArUco Pose Estimation", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.pipeline.stop()
            rclpy.shutdown()
            cv2.destroyAllWindows()


    def publish_tf(self, tvec, rvec, frame):
        # Converti il vettore di rotazione in matrice di rotazione
        R_marker, _ = cv2.Rodrigues(rvec)

        # Usa direttamente la trasformazione rilevata senza correzioni
        R_transformed = R_marker
        T_transformed = tvec.reshape(3,1)


        # Inverti la trasformazione per ottenere la posa della telecamera rispetto al marker
        R_camera = R_transformed.T  # Inverti la rotazione (trasposta della matrice di rotazione)
        T_camera = -R_camera @ T_transformed  # Inverti la traslazione

        current_time = time.time()

        # Aggiungi la trasformazione al buffer
        self.pose_buffer.append((current_time, T_camera.flatten(), R_camera))

        if len(self.pose_buffer) > 1:
            time_diff = self.pose_buffer[-1][0] - self.pose_buffer[-2][0]  # Tempo trascorso dall'ultima rilevazione valida
            self.valid_acquisition_time += time_diff


        # Rimuovi le pose più vecchie di TIME_WINDOW
        self.pose_buffer = [(t, p, r) for t, p, r in self.pose_buffer if current_time - t <= TIME_WINDOW]

        # Se ci sono abbastanza dati, calcola la media
        if len(self.pose_buffer) > 1:
            mean_T = np.mean([p for _, p, _ in self.pose_buffer], axis=0)
            mean_R = np.mean([r for _, _, r in self.pose_buffer], axis=0)

            # Assicura che mean_R sia ortogonale usando SVD
            U, _, Vt = np.linalg.svd(mean_R)
            mean_R = U @ Vt  # Ricostruisci una matrice ortogonale più vicina
        else:
            mean_T = T_camera.flatten()
            mean_R = R_camera

        # Definisci la rotazione aggiuntiva da applicare a camera_color_optical_frame per arrivare a camera_link
        try:
            transform = self.tf_buffer.lookup_transform("camera_color_optical_frame","camera_link", rclpy.time.Time())

            # Estrai la rotazione come quaternione e convertila in matrice di rotazione
            quat = transform.transform.rotation
            R_correction = R.from_quat([quat.x, quat.y, quat.z, quat.w]).as_matrix()


            # Estrai la traslazione
            T_correction = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])
            
            # # Stampa la trasformazione applicata
            # euler_angles = R.from_matrix(R_correction).as_euler('xyz', degrees=True)
            # self.get_logger().info(
            #         f"Applying transformation:\n"
            #         f"Rotation (Euler XYZ) color_optical_frame -> camera_link = X: {euler_angles[0]:.2f}°, Y: {euler_angles[1]:.2f}°, Z: {euler_angles[2]:.2f}°\n"
            #         f"Translation color_optical_frame -> camera_link = X: {T_correction[0]:.4f} m, Y: {T_correction[1]:.4f} m, Z: {T_correction[2]:.4f} m"
            #     )
            
        except Exception as e:
            self.get_logger().error(f"Failed to get transform from camera_color_optical_frame to camera_link: {e}")
            R_correction = np.eye(3)  # Matrice identità come fallback
            T_correction = np.zeros(3)  # Nessuna traslazione come fallback



        # Applica la rotazione aggiuntiva alla rotazione media calcolata
        R_final = mean_R @ R_correction  # Prima ruota secondo la media, poi aggiungi la correzione

        # Calcola la traslazione modificata (se vuoi che sia affetta dalla rotazione)
        T_final = mean_T + T_correction # Applica la traslazione aggiuntiva

        # Converti la nuova rotazione in quaternione
        quaternion = R.from_matrix(R_final).as_quat()

        # Disegna gli assi del frame della camera
        # cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, T_camera, 0.05)

        # Creazione del messaggio di trasformazione
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = FRAME_ARUCO
        t.child_frame_id = FRAME_CAMERA

        t.transform.translation.x = float(T_final[0])
        t.transform.translation.y = float(T_final[1])
        t.transform.translation.z = float(T_final[2])

        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform from {FRAME_CAMERA} to {FRAME_ARUCO}")

    def save_camera_to_base_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(BASE_FRAME, FRAME_CAMERA, rclpy.time.Time())
            
            data = {
                "translation": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z
                },
                "rotation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                }
            }

            with open(calibration_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)

            self.get_logger().info(f"Static transform saved in: {calibration_path}")
        except Exception as e:
            self.get_logger().error(f"No transform available: {e}")
    
class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Action client per il gripper
        self.action_client = ActionClient(self, GripperCommand, '/fr3_gripper/gripper_action')

        # Verifica della disponibilità del server d'azione
        self.get_logger().info("Waiting for gripper action server...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available. Exiting.")
            self.action_client = None  # Imposta a None per evitare ulteriori operazioni
            return

        self.get_logger().info("Gripper action server available.")

    def send_gripper_command(self, position: float, max_effort: float):
        """Invia un comando al gripper."""
        if self.action_client is None:
            self.get_logger().error("Action client not initialized. Cannot send command.")
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position  # Posizione desiderata del gripper
        goal_msg.command.max_effort = max_effort  # Sforzo massimo consentito

        self.get_logger().info(f"Sending gripper command: position={position}, max_effort={max_effort}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Gestisce la risposta del server al comando."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Gripper command rejected.")
                return
            self.get_logger().info("Gripper command accepted. Waiting for result...")
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")

    def result_callback(self, future):
        """Gestisce il risultato del comando inviato al gripper."""
        try:
            result = future.result().result
            if result.stalled:
                self.get_logger().info("Gripper stalled (object likely grasped).")
            elif result.reached_goal:
                self.get_logger().info("Gripper reached goal.")
            else:
                self.get_logger().info("Gripper command executed but no specific result reported.")
        except Exception as e:
            self.get_logger().error(f"Result callback error: {e}")

    def open_gripper(self, position: float = 0.04, max_effort: float = 100.0):
        """Apre il gripper alla posizione specificata con lo sforzo massimo indicato."""
        self.get_logger().info("Fingers opening")
        self.send_gripper_command(position, max_effort)

    def close_gripper(self, position: float = 0.03, max_effort: float = 100.0):
        """Chiude il gripper alla posizione specificata con lo sforzo massimo indicato."""
        self.get_logger().info("Fingers closing")
        self.send_gripper_command(position, max_effort)

    def wait_for_marker(self, wait_time: int = 5):
        """Attende che l'utente inserisca il marker ArUco."""
        self.get_logger().info("Insert the ArUco Marker")
        time.sleep(wait_time)


def main(args=None):
    rclpy.init(args=args)
    # apro gripper per inserire aruco
    node = ArucoPosePublisher()
    gripper = GripperController()
    
    # Se l'action server del gripper non è disponibile, termina il programma
    if gripper.action_client is None:
        node.destroy_node()
        rclpy.shutdown()
        return  # Esce dal main

    # Procedi con il normale flusso se il gripper è disponibile
    gripper.open_gripper()   # Apertura iniziale
    gripper.wait_for_marker()  # Attendi inserimento ArUco
    gripper.close_gripper()  # Chiusura dopo il tempo di attesa

    time.sleep(5)
    node.get_logger().info("Starting acquisition...")

    while rclpy.ok() and node.valid_acquisition_time < TIME_WINDOW:
        rclpy.spin_once(node, timeout_sec=0.1)  # Spin con timeout breve per mantenere il nodo reattivo
        node.get_logger().info(f"Valid acquisition: {node.valid_acquisition_time:.2f}/{TIME_WINDOW} sec.")

    # Una volta che il tempo è sufficiente, salva la trasformata
    node.save_camera_to_base_transform()
    node.get_logger().info("Calibration completed. Node stopped.")

    # Riapertura del gripper prima di terminare**
    if gripper.action_client is not None:
        gripper.open_gripper()  # Riapri il gripper prima di chiudere il nodo
    else:
        gripper.get_logger().error("Gripper action server is unavailable. Skipping final gripper release.")

    gripper.destroy_node()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
