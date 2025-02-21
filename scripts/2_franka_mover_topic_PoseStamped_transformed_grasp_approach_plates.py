#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped,TransformStamped, Quaternion, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, CollisionObject
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from std_msgs.msg import Bool # per avvisare se il robot è ready o busy
from franka_msgs.action import Grasp
from visualization_msgs.msg import Marker
import numpy as np
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory

###### CONFIGURAZIONE GENERALE ###########
# Costanti globali per i reference frame 
START_RF = "camera_link"  # Nome RF centrale videocamera 
TARGET_RF = "fr3_link0"  # Nome RF base del robot
HAND_RF = "fr3_hand_tcp"
DEPTH_VIEW_RF = "camera_depth_optical_frame"  # Nome RF riferimento per le depth images
############ POSIZIONE DI camera_link RISPETTO a fr3_link0 ################
#### Il file di calibrazione viene generato da aruco_calibrate.py e si chiama "fr3_camera_calibration.yaml" nella cartella 
CALIBRATION_FILE = "fr3_camera_calibration.yaml"
calibration_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", CALIBRATION_FILE)# Carica i valori della trasformata dal file YAML, se non li trovo uso i predefiniti
# Se il file non esiste, prova a cercarlo nella cartella installata dopo colcon build
if not os.path.exists(calibration_path):
    try:
        calibration_path = os.path.join(get_package_share_directory('franka_mover'), "config", CALIBRATION_FILE)
    except Exception:
        calibration_path = None  # Se il pacchetto non è installato, il file non esiste
# Se il file di calibrazione non è presente, termina lo script
if not calibration_path or not os.path.exists(calibration_path):
    print("Calibration file is not present in franka_mover/config directory.")
    exit(1)  # Termina lo script con errore

# Caricamento del file di calibrazione
try:
    with open(calibration_path, 'r') as file:
        data = yaml.safe_load(file) or {}
    translation_x = data["translation"]["x"]
    translation_y = data["translation"]["y"]
    translation_z = data["translation"]["z"]
    rotation_x = data["rotation"]["x"]
    rotation_y = data["rotation"]["y"]
    rotation_z = data["rotation"]["z"]
    rotation_w = data["rotation"]["w"]
    print(f"Calibration file: {calibration_path} correctly sourced.")
except Exception as e:
    print(f"Error in {CALIBRATION_FILE}: {e}")
    exit(1)  # Termina lo script con errore
##########################################
# Costanti globali per il rilascio della mela sul cesto (posizione e orientamento di HAND_RF)
# Le posizioni e orientamento sono rispetto a TARGET_RF
to_basket_position_x = 0.4
to_basket_position_y = 0.8
to_basket_position_z = 0.3
#to_basket_orientation_x = 0.0
#to_basket_orientation_y = -np.sqrt(2)/2
#to_basket_orientation_z = 0.0
#to_basket_orientation_w = np.sqrt(2)/2
##########################################
# Costanti globali per la posizione e dimensioni dell'ostacolo BOX rispetto a TARGET_RF
TABLE_SIZE_X = 0.8
TABLE_SIZE_Y = 1.2
TABLE_SIZE_Z = 0.2
TABLE_POS_X = 0.187
TABLE_POS_Y = 0.42
TABLE_POS_Z = -0.100
############################################
##### GRIPPER COSTANTI #########
MAX_EFFORT = 100.0
OPEN = 0.04
CLOSE = 0.03 #da tarare in base alla mela
############################################
#### Pubblicazione cilindro / Sfera ########
############################################
DISTANCE_APPROACH = 0.15

class MoveFR3(Node):
    def __init__(self):
        super().__init__('move_fr3_node')

        # Action client per MoveGroup
        self.action_client = ActionClient(self, MoveGroup, '/move_action')

        # Sottoscrizione al topic /apple_coordinates_realsense
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apple_coordinates_realsense',
            self.apple_callback,
            10  # QoS depth
        )
        self.get_logger().info('Subscribed to /apple_coordinates_realsense')

        # Variabile per memorizzare la posizione target
        self.target_pose = None

        # Pubblicazione del reference frame statico
        self.broadcast_static_transform()

        #Aggiungo buffer per gestire la trasformazione posestamped
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Publisher per pubblicare coordinate mela trasformate su /apple_coordinates_robot (PointStamped)
        self.robot_coordinates_publisher = self.create_publisher(PointStamped, '/apple_coordinates_robot', 10)

        # Action client per il gripper
        self.gripper_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        self.status_publisher = self.create_publisher(Bool, '/robot_status', 10)
        self.robot_is_ready = True  # Inizialmente il robot è pronto
        self.status_publisher.publish(Bool(data=self.robot_is_ready))

        # Publisher per gli oggetti di collisione di MoveIt
        self.collision_object_publisher = self.create_publisher(
            CollisionObject, 
            "/collision_object",
            #"/planning_scene", 
            10
        )

        # Verifica della disponibilità del server d'azione
        self.get_logger().info("Waiting for grasp action server...")
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Grasp action server not available.")
            self.gripper_client = None  # Imposta a None per evitare ulteriori operazioni
        else:
            self.get_logger().info("Grasp action server available.")


        # Creazione e pubblicazione dell'oggetto di collisione tavolo 
        collision_table = self.add_collision_box(TABLE_SIZE_X, TABLE_SIZE_Y, TABLE_SIZE_Z,TABLE_POS_X, TABLE_POS_Y, TABLE_POS_Z, "collision_table")
        # Creazione oggetto di collisione robot
        collision_robot = self.add_collision_box(0.29, 0.29, 0.6, 0.0, 0.85, 0.30,"collision_robot")
        # Creazione oggetto di collisione cesto
        collision_basket = self.add_collision_box(0.4, 0.4, 0.15, 0.387, 0.82, 0.075,"collision_basket")
        # Creo Muro lavagna collisione
        collision_wall = self.add_collision_box(0.05, 2.0, 2.0 , -0.5, 0.0 , 0.0 , "collision_wall")
        # Creo Scrivania Collisione
        collision_desk = self.add_collision_box(2.0 , 0.05, 2.0, 0.0 , -0.5 , 0.0 ,"collision_desk")

        # Attendo un po' per assicurarsi che MoveIt riceva il messaggio
        time.sleep(1.0)
        self.get_logger().info("Publishing collision objects...")
        self.collision_object_publisher.publish(collision_table)
        time.sleep(1.0)
        self.collision_object_publisher.publish(collision_robot)
        time.sleep(1.0)
        self.collision_object_publisher.publish(collision_basket)
        time.sleep(1.0)
        time.sleep(1.0)
        self.collision_object_publisher.publish(collision_wall)
        time.sleep(1.0)
        self.collision_object_publisher.publish(collision_desk)
        time.sleep(1.0)
        self.get_logger().info("Collision objects published!")

    def broadcast_static_transform(self):
        """Definisce e pubblica la trasformazione statica tra START_RF e TARGET_RF."""
        static_broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = TARGET_RF
        static_transform.child_frame_id = START_RF

        # Imposta traslazione
        static_transform.transform.translation.x = translation_x
        static_transform.transform.translation.y = translation_y
        static_transform.transform.translation.z = translation_z

        # Imposta rotazione
        static_transform.transform.rotation.x = rotation_x
        static_transform.transform.rotation.y = rotation_y
        static_transform.transform.rotation.z = rotation_z
        static_transform.transform.rotation.w = rotation_w

        # Pubblica la trasformazione statica
        static_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f"Static transform published: {START_RF} -> {TARGET_RF}")

    def add_collision_box(self, size_x, size_y, size_z, pos_x, pos_y, pos_z, name):
        """
        Crea e restituisce un oggetto di collisione di tipo BOX per MoveIt.

        Args:
            size_x (float): Dimensione lungo l'asse X.
            size_y (float): Dimensione lungo l'asse Y.
            size_z (float): Dimensione lungo l'asse Z.
            pos_x (float): Posizione lungo l'asse X.
            pos_y (float): Posizione lungo l'asse Y.
            pos_z (float): Posizione lungo l'asse Z.

        Returns:
            CollisionObject: Oggetto di collisione pronto per essere aggiunto alla scena di MoveIt.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = TARGET_RF
        collision_object.id = name

        # Definizione della forma del BOX
        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [size_x, size_y, size_z]

        # Definizione della posa del BOX
        box_pose = Pose()
        box_pose.position.x = pos_x
        box_pose.position.y = pos_y
        box_pose.position.z = pos_z
        box_pose.orientation.w = 1.0  # Nessuna rotazione

        # Aggiunge la forma e la posa all'oggetto di collisione
        collision_object.primitives.append(box_primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        return collision_object
    
    def add_collision_apple(self, pos_x , pos_y , pos_z , reference_frame = TARGET_RF, radius = 0.03, plate_height = 0.01, plate_radius = 0.11, plate_distance = 0.015):
        """
        Aggiunge una sfera con due piatti tangenti sopra e sotto come oggetti di collisione nell'ambiente di MoveIt.

        Args:
            radius (float): Raggio della sfera.
            plate_height (float): Altezza dei piatti circolari.
            plate_radius (float): Raggio dei piatti circolari.
            pos_x (float): Coordinata X della posizione.
            pos_y (float): Coordinata Y della posizione.
            pos_z (float): Coordinata Z della posizione.
            reference_frame (str): Nome del sistema di riferimento.
        """
        # Creazione della sfera
        collision_sphere = CollisionObject()
        collision_sphere.header.frame_id = reference_frame
        collision_sphere.id = "collision_sphere"

        sphere_primitive = SolidPrimitive()
        sphere_primitive.type = SolidPrimitive.SPHERE
        sphere_primitive.dimensions = [radius]  # Il raggio della sfera

        sphere_pose = Pose()
        sphere_pose.position.x = pos_x
        sphere_pose.position.y = pos_y
        sphere_pose.position.z = pos_z
        sphere_pose.orientation.w = 1.0  

        collision_sphere.primitives.append(sphere_primitive)
        collision_sphere.primitive_poses.append(sphere_pose)
        collision_sphere.operation = CollisionObject.ADD
        
        self.collision_object_publisher.publish(collision_sphere)
        time.sleep(1.0)
        self.get_logger().info(f"Added collision sphere in frame '{reference_frame}' with radius {radius} m.")

        # Creazione dei due piatti
        for i, offset in enumerate([-(radius + plate_distance), (radius + plate_distance)]):
            collision_plate = CollisionObject()
            collision_plate.header.frame_id = reference_frame
            collision_plate.id = f"collision_plate_{i}"

            plate_primitive = SolidPrimitive()
            plate_primitive.type = SolidPrimitive.CYLINDER
            plate_primitive.dimensions = [plate_height, plate_radius]  # Altezza, raggio

            plate_pose = Pose()
            plate_pose.position.x = pos_x
            plate_pose.position.y = pos_y
            plate_pose.position.z = pos_z + offset  # Posizionati sopra e sotto la sfera
            plate_pose.orientation.w = 1.0  

            collision_plate.primitives.append(plate_primitive)
            collision_plate.primitive_poses.append(plate_pose)
            collision_plate.operation = CollisionObject.ADD
            
            self.collision_object_publisher.publish(collision_plate)
            self.get_logger().info(f"Added collision plate {i} in frame '{reference_frame}' with radius {plate_radius} m and height {plate_height} m.")

    def remove_collision_apple(self, sphere_id="collision_sphere", plate_ids=["collision_plate_0", "collision_plate_1"]):
        """
        Rimuove la sfera e i due piatti dall'ambiente MoveIt usando i loro ID se sono stati pubblicati.

        Args:
            sphere_id (str): L'ID dell'oggetto sfera da rimuovere.
            plate_ids (list of str): Lista degli ID dei piatti da rimuovere.
        """
        # Rimozione della sfera
        collision_object = CollisionObject()
        collision_object.id = sphere_id
        collision_object.header.frame_id = TARGET_RF
        collision_object.operation = CollisionObject.REMOVE  
        self.collision_object_publisher.publish(collision_object)
        self.get_logger().info(f"Object '{sphere_id}' removed.")

        # Rimozione dei piatti
        for plate_id in plate_ids:
            collision_object = CollisionObject()
            collision_object.id = plate_id
            collision_object.header.frame_id = TARGET_RF
            collision_object.operation = CollisionObject.REMOVE  
            self.collision_object_publisher.publish(collision_object)
            self.get_logger().info(f"Object '{plate_id}' removed.")

    def apple_callback(self, msg: PoseStamped):
        """Callback per il topic /apple_coordinates_realsense."""

        # log terminale del punto ( ancora da trasformare)
        self.get_logger().info(f"Apple position Realsense Frame: x={msg.pose.position.x:.5f}, "
                       f"y={msg.pose.position.y:.5f}, z={msg.pose.position.z:.5f}")

        try:
            # Aspetta la trasformazione dal frame sorgente al frame TARGET_RF
            transform = self.tf_buffer.lookup_transform(
                TARGET_RF,  # Frame di destinazione
                msg.header.frame_id,  # Frame sorgente (es. DEPTH_VIEW_RF)
                rclpy.time.Time(),  # Tempo della trasformazione (usa il timestamp del messaggio)
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            # Trasforma il messaggio PoseStamped
            transformed_pose = do_transform_pose_stamped(msg, transform)

            # Log del risultato trasformato
            self.get_logger().info(f"Apple position Robot Frame: x={transformed_pose.pose.position.x:.5f}, "
                       f"y={transformed_pose.pose.position.y:.5f}, z={transformed_pose.pose.position.z:.5f}")
            
            #per aggiungere la sfera di collisione (apple)
            self.add_collision_apple(transformed_pose.pose.position.x,transformed_pose.pose.position.y,transformed_pose.pose.position.z)
            
            # Creazione del messaggio PointStamped da inviare a /apple_coordinates_robot
            point_msg = PointStamped()
            point_msg.header = transformed_pose.header
            point_msg.point = transformed_pose.pose.position

            # Pubblica il messaggio
            self.robot_coordinates_publisher.publish(point_msg)

            # Memorizza la posa trasformata
            self.target_pose = transformed_pose
            

            # Invia il goal
            self.approach_goal()

        except Exception as e:
            self.get_logger().error(f"Error during pose transformation: {str(e)}")

    def obtain_quaternion(self, frame_name: str, rotation_angle_deg: float, vector_x: float, vector_y: float, vector_z: float) -> Quaternion:
        """
        Calcola un quaternione che rappresenta:
        - Rotazione attorno all'asse Y di un angolo specificato.
        - Rotazione attorno all'asse Z di un angolo derivato dall'orientamento del vettore rispetto all'asse X.
        
        Args:
            frame_name (str): Nome del sistema di riferimento.
            rotation_angle_deg (float): Angolo di rotazione attorno all'asse Y in gradi.
            vector_x (float): Componente X del vettore.
            vector_y (float): Componente Y del vettore.
            vector_z (float): Componente Z del vettore.
        
        Returns:
            Quaternion: Quaternione calcolato.
        """
        # Step 1: Calcola l'angolo del vettore rispetto all'asse X (Csi)
        magnitude = np.sqrt(vector_x**2 + vector_y**2 + vector_z**2)
        if magnitude == 0:
            raise ValueError("Il vettore fornito ha magnitudine zero.")
        
        # Proiezione del vettore nel piano XY
        Csi = np.arctan2(vector_y, vector_x)  # Angolo rispetto all'asse X in radianti

        # Step 2: Rotazione attorno all'asse Y
        rotation_angle_rad = np.deg2rad(rotation_angle_deg)  # Converti l'angolo in radianti
        Ry = np.array([
            [np.cos(rotation_angle_rad), 0, np.sin(rotation_angle_rad)],
            [0, 1, 0],
            [-np.sin(rotation_angle_rad), 0, np.cos(rotation_angle_rad)]
        ])

        # Step 3: Rotazione attorno all'asse Z di Csi
        Rz = np.array([
            [np.cos(Csi), -np.sin(Csi), 0],
            [np.sin(Csi), np.cos(Csi), 0],
            [0, 0, 1]
        ])

        # Step 4: Combinazione delle rotazioni
        R = Rz @ Ry  # Applica prima la rotazione attorno a Y, poi attorno a Z

        # Step 5: Calcola il quaternione corrispondente
        qw = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
        qx = (R[2, 1] - R[1, 2]) / (4 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4 * qw)

        # Step 6: Restituisci il quaternione in formato ROS2
        quaternion = Quaternion()
        quaternion.x = qx
        quaternion.y = qy
        quaternion.z = qz
        quaternion.w = qw

        return quaternion
    
    def make_constraints(self,pos_x, pos_y, pos_z, orient_x=None, orient_y=None, orient_z=None, orient_w=None):
        """
        Crea e restituisce vincoli di posizione e orientamento per il movimento del robot.

        Args:
            pos_x, pos_y, pos_z (float): Coordinate della posizione target.
            orient_x, orient_y, orient_z, orient_w (float): Quaternione di orientamento (se presente)
            gli oggetti di collisione sono automaticamente presi in considerazione da moveit !

        Returns:
            Constraints: Oggetto contenente vincoli di posizione, orientamento (se presente).
            I vincoli di collisione sono gia tenuti conto dalla scena moveit!
        """
        constraints = Constraints()

        # Vincolo di posizione
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = TARGET_RF
        position_constraint.link_name = HAND_RF
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.001, 0.001, 0.001])
        )

        position_pose = PoseStamped()
        position_pose.header.frame_id = TARGET_RF
        position_pose.pose.position.x = pos_x
        position_pose.pose.position.y = pos_y
        position_pose.pose.position.z = pos_z

        position_constraint.constraint_region.primitive_poses.append(position_pose.pose)
        position_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)

        # Vincolo di orientamento
        if None not in (orient_x, orient_y, orient_z, orient_w):
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = TARGET_RF
            orientation_constraint.link_name = HAND_RF
            orientation_constraint.orientation.x = orient_x
            orientation_constraint.orientation.y = orient_y
            orientation_constraint.orientation.z = orient_z
            orientation_constraint.orientation.w = orient_w
            orientation_constraint.absolute_x_axis_tolerance = 0.01
            orientation_constraint.absolute_y_axis_tolerance = 0.01
            orientation_constraint.absolute_z_axis_tolerance = 0.01
            orientation_constraint.weight = 1.0
            
            constraints.orientation_constraints.append(orientation_constraint)

        return constraints
    
    def send_grasp_command(self, position: float, effort: float, innerepsilon = 0.5, outerepsilon = 0.5, velocity = 0.2):
        """Invia un comando al gripper."""
        if self.action_client is None:
            self.get_logger().error("Action client not initialized. Cannot send command.")
            return

        goal_msg = Grasp.Goal()
        goal_msg.epsilon.inner = innerepsilon  
        goal_msg.epsilon.outer = outerepsilon  
        goal_msg.force = effort
        goal_msg.speed = velocity
        goal_msg.width = position

        self.get_logger().info(f"Sending grasp command: width={position}, effort={effort}, speed={velocity}, epsilonIN/OUT={innerepsilon},{outerepsilon}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.grasp_response_callback)

    def grasp_response_callback(self, future):
        """Gestisce la risposta del server al comando."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Grasp command rejected.")
                return
            self.get_logger().info("Grasp command accepted. Waiting for result...")
            goal_handle.get_result_async().add_done_callback(self.grasp_result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")

    def grasp_result_callback(self, future):
        """Gestisce il risultato del comando inviato al gripper."""
        try:
            result = future.result().result
            if result.stalled:
                self.get_logger().info("Grasp stalled (object likely grasped).")
            elif result.reached_goal:
                self.get_logger().info("Grasp reached goal.")
            else:
                self.get_logger().info("Grasp command executed but no specific result reported.")
        except Exception as e:
            self.get_logger().error(f"Result grasp callback error: {e}")

    def approach_goal(self):
        " 1 APPROCCIO INIZIALE "
        """Invia un goal al server MoveGroup utilizzando la posizione target ricevuta."""
        "Prima si approccia a una distanza DISTANCE_APPROACH, poi si calcolano waypoint e si va avanti per quei waypoints"

        self.robot_is_ready = False #il robot non è piu pronto, ma busy
        self.status_publisher.publish(Bool(data=self.robot_is_ready))


        if not self.target_pose:
            self.get_logger().warn('No target pose approach received yet. Waiting...')
            return

        self.get_logger().info('Waiting for move_group approach action server...')
        self.action_client.wait_for_server()

        # Creazione del messaggio di goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'

        # Configurazione della pose target direttamente dalla posa trasformata
        target_pose = self.target_pose
        target_pose.header.frame_id = TARGET_RF  # Assicurarsi che il frame sia corretto

        # Orientamento del end-effector
        quaternion = self.obtain_quaternion(
                target_pose.header.frame_id, 
                90,
                target_pose.pose.position.x,
                target_pose.pose.position.y, 
                target_pose.pose.position.z )
                
        # Crea vincoli di movimento 
        constraints = self.make_constraints(
                target_pose.pose.position.x - DISTANCE_APPROACH * np.cos(np.arctan2(target_pose.pose.position.y, target_pose.pose.position.x)),
                target_pose.pose.position.y - DISTANCE_APPROACH * np.sin(np.arctan2(target_pose.pose.position.y, target_pose.pose.position.x)),
                target_pose.pose.position.z,
                quaternion.x, 
                quaternion.y, 
                quaternion.z, 
                quaternion.w
                )
        
        # Imposta i vincoli come obiettivo
        goal_msg.request.goal_constraints.append(constraints)

        # APPLICA tolleranze di replanning
        goal_msg.request.allowed_planning_time= 5.0  # Tempo massimo di pianificazione
        goal_msg.planning_options.replan_attempts = 50
        goal_msg.planning_options.replan = True  # Abilita il ricalcolo del percorso se fallisce

        # goal_msg.request.planner_id = ""  # Usa il planner predefinito, che tiene conto delle collisioni
        # Invio del goal
        self.get_logger().info('Sending goal approach to move_group...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.approach_response_callback)

    def approach_response_callback(self, future):
        """Gestisce la risposta del server MoveGroup al approach inviato."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal approach rejected by move_group.')
                return
            self.get_logger().info('Goal approach accepted by move_group. Waiting for result...')
            goal_handle.get_result_async().add_done_callback(self.approach_result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal approach response error: {str(e)}")
    
    def approach_result_callback(self, future):
        """Gestisce il risultato del goal approach inviato."""
        try:
            result = future.result().result
            if result.error_code.val == 1:  # 1 indica SUCCESSO nella MoveIt error codes
                self.get_logger().info('Goal approach reached successfully!')
                self.get_logger().info('Need to advance the apple')
                
                #FACCIO L'ADVANCE
                self.advance_goal()

            else:
                self.get_logger().error(f'MoveGroup approach failed with error code: {result.error_code.val}')
               
                # il robot NON riesce ad andare in quella posizione di approach quindi è di nuovo libero
                # attendo prossime coordinate
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))
                self.remove_collision_apple()

        except Exception as e:
            self.get_logger().error(f"Result approach callback error: {str(e)}")

            # il robot NON riesce ad andare in quella posizione di approach quindi è di nuovo libero
            # attendo nuove coordinate
            self.robot_is_ready = True
            self.status_publisher.publish(Bool(data=self.robot_is_ready))
            self.remove_collision_apple()

    def advance_goal(self):
        """Fase 2: Movimento diretto in linea retta verso la mela senza step intermedi."""
        target_pose = self.target_pose

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'

        quaternion = self.obtain_quaternion(
            TARGET_RF,
            90,
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z
        )

        # Imposta vincoli di posizione e orientamento
        constraints = self.make_constraints(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        )

        goal_msg.request.goal_constraints.append(constraints)

        # APPLICA tolleranze di replanning
        goal_msg.request.allowed_planning_time= 10.0  # Tempo massimo di pianificazione
        goal_msg.planning_options.replan_attempts = 50
        goal_msg.planning_options.replan = True  # Abilita il ricalcolo del percorso se fallisce

        self.get_logger().info('Sending direct goal to move_group...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.advance_response_callback)

    def advance_response_callback(self, future):
        """Gestisce la risposta del server MoveGroup advance al goal inviato."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal advance rejected by move_group.')
                return
            self.get_logger().info('Goal advance accepted by move_group. Waiting for result...')
            goal_handle.get_result_async().add_done_callback(self.advance_result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal advance response error: {str(e)}")

    def advance_result_callback(self, future):
        """Gestisce il risultato del goal advance inviato."""
        try:
            result = future.result().result
            if result.error_code.val == 1:  # 1 indica SUCCESSO nella MoveIt error codes
                self.remove_collision_apple()
                self.get_logger().info('Apple goal reached successfully!')
                self.get_logger().info('Need to grasp the apple')
                
                # FACCIO IL GRASPING
                self.send_grasp_command(position = CLOSE, effort = MAX_EFFORT)  # Chiudi il gripper
                self.get_logger().info('Grasping...')
                time.sleep(2.0)
                self.get_logger().info('Fingers closed: apple grasped')

                #MANDO ROBOT AL CESTO
                self.go_to_basket() 

            else:
                self.get_logger().error(f'MoveGroup failed advance with error code: {result.error_code.val}')
               
                # il robot NON riesce ad andare in quella posizione quindi è di nuovo libero
                # attendo prossime coordinate
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))
                self.remove_collision_apple()

        except Exception as e:
            self.get_logger().error(f"Result advance callback error: {str(e)}")

            # il robot NON riesce ad andare in quella posizione quindi è di nuovo libero
            # attendo nuove coordinate
            self.robot_is_ready = True
            self.status_publisher.publish(Bool(data=self.robot_is_ready))
            self.remove_collision_apple()

    def go_to_basket(self):
        """Sposta il robot alla posizione del basket specificata nelle costanti."""
        self.get_logger().info('Moving to basket...')
        basket_goal = MoveGroup.Goal()
        basket_goal.request.group_name = 'fr3_arm'

        # Crea vincoli di movimento per la posizione del cesto
        constraints = self.make_constraints(
                to_basket_position_x,
                to_basket_position_y,
                to_basket_position_z,
            )

        basket_goal.request.goal_constraints.append(constraints)

        # APPLICA tolleranze di replanning
        basket_goal.request.allowed_planning_time= 5.0  # Tempo massimo di pianificazione
        basket_goal.planning_options.replan_attempts = 30
        basket_goal.planning_options.replan = True  # Abilita il ricalcolo del percorso se fallisce

        # Invio del goal
        self.get_logger().info('Sending goal to basket...')
        future = self.action_client.send_goal_async(basket_goal)
        future.add_done_callback(self.basket_response_callback)

    def basket_response_callback(self, future):
        """Gestisce la risposta del server MoveGroup al goal per il basket."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Basket goal rejected.')
                return
            self.get_logger().info('Basket goal accepted. Waiting for result...')
            goal_handle.get_result_async().add_done_callback(self.basket_result_callback)
        except Exception as e:
            self.get_logger().error(f"Error sending basket goal: {e}")

    def basket_result_callback(self, future):
        """Gestisce il risultato del goal per il basket."""
        try:
            result = future.result().result
            if result.error_code.val == 1:  # Success
                self.get_logger().info('Basket goal reached successfully!')

                # RILASCIO LA MELA
                self.send_grasp_command(position = OPEN, effort = MAX_EFFORT)  # Apri il gripper
                self.get_logger().info('Releasing...')
                time.sleep(2.0)
                self.get_logger().info('Fingers opened: apple released')

                # il robot ha raggiunto l'obiettivo ed è libero
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))      

            else:
                self.get_logger().error(f'Basket goal failed with error code: {result.error_code.val}')
                self.get_logger().error(f'Robot grasped the apple, but can''t go to the basket: {result.error_code.val}')

                ## LA MELA DEVE ESSERE RILASCIATA
                # RILASCIO LA MELA
                self.send_grasp_command(position = OPEN, effort = MAX_EFFORT)  # Apri il gripper
                self.get_logger().info('Releasing...')
                time.sleep(2.0)
                self.get_logger().info('Fingers opened: apple released')

                # rimetto il robot libero
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))

        except Exception as e:
            self.get_logger().error(f"Error receiving basket goal result: {e}")
            self.get_logger().error(f'Robot grasped the apple, but can''t go to the basket: {result.error_code.val}')
            
            ## LA MELA DEVE ESSERE RILASCIATA
            # RILASCIO LA MELA
            self.send_grasp_command(position = OPEN, effort = MAX_EFFORT)  # Apri il gripper
            self.get_logger().info('Releasing...')
            time.sleep(2.0)
            self.get_logger().info('Fingers opened: apple released')


            # rimetto il robot libero
            self.robot_is_ready = True
            self.status_publisher.publish(Bool(data=self.robot_is_ready))



def main(args=None):
    rclpy.init(args=args)
    node = MoveFR3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()