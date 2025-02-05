import numpy as np
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
import time
import yaml
import os

CALIBRATION_FILE = "fr3_camera_calibration.yaml"
ARUCO_SIZE = 0.1  # Dimensione del codice ArUco (10 cm)
DISPLAY_SCALE = 0.7  # Fattore di riduzione della finestra di visualizzazione
TIME_WINDOW = 20.0  # Secondi per la media mobile
BASE = "fr3_link0"
CAMERA_BASE = "camera_link"
END_EFFECTOR = "fr3_hand_tcp"
ARUCO = "aruco_marker"

# Aruco rispetto a fr3_hand_tcp con rotazione X->Y->Z e traslazione(da misurare manualmente)
# Sono date dal supporto montato con faccia dove ce il punto rosso
X_ROT = -90.0  
Y_ROT = -90.0
Z_ROT = 0.0  
X_TRANSLATION = -0.009  # Traslazione -9mm per portarla al pari con filo gommini
Y_TRANSLATION = 0.0
Z_TRANSLATION = 0.0

# Aruco code è visto da camera_color_optical_frame 
# Bisogna calcolare la trasformazione inversa che porta da camera_color_optical_frame -> camera_link
# ros2 run tf2_ros tf2_echo camera_color_optical_frame camera_link

class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        self.finished = False # per uscire prog

        # Publisher per la trasformazione ArUco → camera
        self.pose_pub = self.create_publisher(TransformStamped, '/aruco_pose', 10)

        # Broadcaster per le trasformazioni TF2
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Pubblica la trasformazione statica tra fr3_hand_tcp e aruco_marker
        self.pub_static_fr3_aruco()

        # Dizionario ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        # Bridge per conversione immagine ROS → OpenCV
        self.bridge = CvBridge()

        # Sottoscrizione ai topic della telecamera con QoS per minore latenza
        qos = rclpy.qos.qos_profile_sensor_data

        self.rgb_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.rgb_callback, qos)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, qos)

        # Variabili per immagazzinare i dati della telecamera
        self.rgb_image = None
        self.intrinsics = None

        # Buffer per la media delle trasformazioni
        self.transform_buffer = []  # Lista di tuple (timestamp, posizione, orientazione)

        # Timer per pubblicare la trasformazione statica ogni 5 secondi
        self.create_timer(TIME_WINDOW, self.publish_transform_aruco_camera)

        # Buffer e Listener per TF2 (necessari per ottenere trasformazioni esistenti)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def pub_static_fr3_aruco(self):
        """Pubblica una trasformazione statica tra fr3_hand_tcp e aruco_marker."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = END_EFFECTOR  
        static_transform.child_frame_id = ARUCO

        # Posizione fissa rispetto al TCP (da calibrare)
        static_transform.transform.translation.x = X_TRANSLATION  
        static_transform.transform.translation.y = Y_TRANSLATION
        static_transform.transform.translation.z = Z_TRANSLATION

        # Calcola i quaternioni dalla rotazione specificata
        rotation = R.from_euler('XYZ', [X_ROT, Y_ROT, Z_ROT], degrees=True)
        quat = rotation.as_quat()

        # Assegna i valori alla trasformazione
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("[TF] Pubblicata trasformazione statica: fr3_hand_tcp → aruco_marker")


        
    def rgb_callback(self, msg):
        """Elabora il frame RGB solo se gli intrinseci sono disponibili."""
        if self.intrinsics is None:
            return

        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.process_frame()
        except Exception as e:
            self.get_logger().error(f"Errore nella conversione RGB: {e}")

    def camera_info_callback(self, msg):
        """Memorizza i parametri intrinseci della telecamera."""
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0], 'fy': msg.k[4], 'ppx': msg.k[2], 'ppy': msg.k[5]
            }
            self.get_logger().info("[INFO] Parametri intrinseci ricevuti.")

    def process_frame(self):
        """Rileva il marker ArUco e salva la trasformazione in un buffer per la media mobile."""
        if self.rgb_image is None:
            return

        gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            aruco.drawDetectedMarkers(self.rgb_image, corners, ids)

            camera_matrix = np.array([
                [self.intrinsics['fx'], 0, self.intrinsics['ppx']],
                [0, self.intrinsics['fy'], self.intrinsics['ppy']],
                [0, 0, 1]
            ])
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE, camera_matrix, np.zeros(5))

            for rvec, tvec in zip(rvecs, tvecs):
                cv2.drawFrameAxes(self.rgb_image, camera_matrix, np.zeros(5), rvec, tvec, 0.03)

                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_matrix).as_quat()

                # Aggiungo la nuova trasformazione al buffer
                timestamp = time.time()
                self.transform_buffer.append((timestamp, tvec.flatten(), quat))

                # Rimuovi trasformazioni più vecchie di 5 secondi
                self.transform_buffer = [
                    (t, pos, rot) for t, pos, rot in self.transform_buffer if timestamp - t <= TIME_WINDOW
                ]

        display_image = cv2.resize(self.rgb_image, (0, 0), fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)
        cv2.imshow("Aruco Tracking", display_image)
        cv2.waitKey(1)

    def publish_transform_aruco_camera(self):
        """Calcola la media delle ultime trasformazioni e pubblica come statica."""
        if not self.transform_buffer:
            self.get_logger().warn("[TF] Nessuna trasformazione disponibile per la media")
            return

        positions = np.array([pos for _, pos, _ in self.transform_buffer])
        orientations = np.array([rot for _, _, rot in self.transform_buffer])

        mean_position = np.mean(positions, axis=0)
        mean_orientation = np.mean(orientations, axis=0)

        static_transform_aruco = TransformStamped()
        static_transform_aruco.header.stamp = self.get_clock().now().to_msg()
        static_transform_aruco.header.frame_id = ARUCO
        static_transform_aruco.child_frame_id = CAMERA_BASE

        static_transform_aruco.transform.translation.x = mean_position[0]  #- 0.015
        static_transform_aruco.transform.translation.y = mean_position[1]  #- 0.015
        static_transform_aruco.transform.translation.z = mean_position[2]  #- 0.015

        # Converti la rotazione calcolata in un oggetto Rotation
        original_rotation = R.from_quat(mean_orientation)

        # Crea la rotazione aggiuntiva (DA RIVEDERE)
        additional_rotation = R.from_euler('YXZ', [81.650, 89.729, -8.398], degrees=True) 

        # Componi le due rotazioni
        final_rotation = additional_rotation * original_rotation

        final_quaternion = final_rotation.as_quat()

        # Assegna la rotazione modificata alla trasformazione
        static_transform_aruco.transform.rotation.x = final_quaternion[0]
        static_transform_aruco.transform.rotation.y = final_quaternion[1]
        static_transform_aruco.transform.rotation.z = final_quaternion[2]
        static_transform_aruco.transform.rotation.w = final_quaternion[3]
        
        # Pubblica la trasformazione statica aruco_marker → camera_link
        self.static_tf_broadcaster.sendTransform(static_transform_aruco)
        self.get_logger().info(f"[TF] Pubblicata trasformazione statica:{ARUCO} → {CAMERA_BASE}")
        # self.retrieve_and_save_fr3_to_camera()
        self.finished = True


    def retrieve_and_save_fr3_to_camera(self):
        """Ascolta la trasformata `fr3_link0 → camera_link` e la salva in `CALIBRATION_FILE`."""
        try:
            # Attendi finché la trasformata non è disponibile
            self.get_logger().info(f"[TF] In ascolto della trasformata {BASE} → {CAMERA_BASE}...")
            transform = self.tf_buffer.lookup_transform(BASE, CAMERA_BASE, rclpy.time.Time(), rclpy.duration.Duration(seconds=5.0))

            # Prepara i dati per il salvataggio
            data = {
                "translation": {axis: getattr(transform.transform.translation, axis) for axis in 'xyz'},
                "rotation": {axis: getattr(transform.transform.rotation, axis) for axis in 'xyzw'}
            }

            # Salva su file YAML
            with open(CALIBRATION_FILE, 'w') as file:
                yaml.dump(data, file)

            self.get_logger().info(f"[TF] Trasformata {BASE} → {CAMERA_BASE} salvata in {CALIBRATION_FILE}")

        except Exception as e:
            self.get_logger().error(f"Errore nel recupero della trasformata {BASE} → {CAMERA_BASE}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    calculated = 25000  # Durata in secondi
    while (not (node.finished) ) and rclpy.ok():
        rclpy.spin_once(node)   
    start_time = time.time()  # Registra il tempo di inizio
    while (time.time() - start_time) < 1 and rclpy.ok():
        rclpy.spin_once(node)  
    node.retrieve_and_save_fr3_to_camera() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
