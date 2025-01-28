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
import numpy as np
import time


# Costanti globali per i reference frame 
BASE_RF = "base"  # Nome RF base del robot
HAND_RF = "fr3_hand_tcp"
# DEPTH_VIEW_RF = "camera_depth_optical_frame"  # Nome RF riferimento per le depth images
##########################################
# Costanti globali per il rilascio della mela sul cesto (posizione e orientamento di HAND_RF)
# Le posizioni e orientamento sono rispetto a BASE_RF
to_basket_position_x = -0.5
to_basket_position_y = 0.0
to_basket_position_z = 0.2
to_basket_orientation_x = 0.0
to_basket_orientation_y = -np.sqrt(2)/2
to_basket_orientation_z = 0.0
to_basket_orientation_w = np.sqrt(2)/2
##########################################
# Costanti globali per la posizione e dimensioni dell'ostacolo BOX rispetto a BASE_RF
OBSTACLE_SIZE_X = 0.8
OBSTACLE_SIZE_Y = 1.2
OBSTACLE_SIZE_Z = 0.2
OBSTACLE_POS_X = 0.0
OBSTACLE_POS_Y = 0.0
OBSTACLE_POS_Z = -0.1
############################################


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

        #Aggiungo buffer per gestire la trasformazione posestamped
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Publisher per pubblicare coordinate mela trasformate su /apple_coordinates_robot (PointStamped)
        self.robot_coordinates_publisher = self.create_publisher(PointStamped, '/apple_coordinates_robot', 10)

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

        # Creazione e pubblicazione dell'oggetto di collisione
        collision_table = self.add_collision_object(
            OBSTACLE_SIZE_X, OBSTACLE_SIZE_Y, OBSTACLE_SIZE_Z, 
            OBSTACLE_POS_X, OBSTACLE_POS_Y, OBSTACLE_POS_Z
        )

        # Attendo un po' per assicurarsi che MoveIt riceva il messaggio
        time.sleep(0.5)
        self.get_logger().info("Publishing collision object...")
        self.collision_object_publisher.publish(collision_table)
        time.sleep(1.0)
        self.get_logger().info("Collision object published!")
    
    def add_collision_object(self, size_x, size_y, size_z, pos_x, pos_y, pos_z):
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
        collision_object.header.frame_id = BASE_RF
        collision_object.id = "collision_table"

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

    def apple_callback(self, msg: PoseStamped):
        """Callback per il topic /apple_coordinates_realsense."""

        # log terminale del punto ( ancora da trasformare)
        self.get_logger().info(f"Apple position Realsense Frame: x={msg.pose.position.x:.5f}, "
                       f"y={msg.pose.position.y:.5f}, z={msg.pose.position.z:.5f}")

        try:
            # Aspetta la trasformazione dal frame sorgente al frame BASE_RF
            transform = self.tf_buffer.lookup_transform(
                BASE_RF,  # Frame di destinazione
                msg.header.frame_id,  # Frame sorgente (es. DEPTH_VIEW_RF)
                rclpy.time.Time(),  # Tempo della trasformazione (usa il timestamp del messaggio)
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            # Trasforma il messaggio PoseStamped
            transformed_pose = do_transform_pose_stamped(msg, transform)

            # Log del risultato trasformato
            self.get_logger().info(f"Apple position Robot Frame: x={transformed_pose.pose.position.x:.5f}, "
                       f"y={transformed_pose.pose.position.y:.5f}, z={transformed_pose.pose.position.z:.5f}")
            
            # Creazione del messaggio PointStamped da inviare a /apple_coordinates_robot
            point_msg = PointStamped()
            point_msg.header = transformed_pose.header
            point_msg.point = transformed_pose.pose.position

            # Pubblica il messaggio
            self.robot_coordinates_publisher.publish(point_msg)


            # Memorizza la posa trasformata
            self.target_pose = transformed_pose

            # Invia il goal
            self.send_goal()

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
    
    def make_constraints(self,pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
        """
        Crea e restituisce vincoli di posizione e orientamento per il movimento del robot.

        Args:
            pos_x, pos_y, pos_z (float): Coordinate della posizione target.
            orient_x, orient_y, orient_z, orient_w (float): Quaternione di orientamento.
            gli oggetti di collisione sono automaticamente presi in considerazione da moveit !

        Returns:
            Constraints: Oggetto contenente vincoli di posizione, orientamento.
            I vincoli di collisione sono gia tenuti conto dalla scena moveit!
        """
        constraints = Constraints()

        # Vincolo di posizione
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = BASE_RF
        position_constraint.link_name = HAND_RF
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        )

        position_pose = PoseStamped()
        position_pose.header.frame_id = BASE_RF
        position_pose.pose.position.x = pos_x
        position_pose.pose.position.y = pos_y
        position_pose.pose.position.z = pos_z

        position_constraint.constraint_region.primitive_poses.append(position_pose.pose)
        position_constraint.weight = 1.0

        # Vincolo di orientamento
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = BASE_RF
        orientation_constraint.link_name = HAND_RF
        orientation_constraint.orientation.x = orient_x
        orientation_constraint.orientation.y = orient_y
        orientation_constraint.orientation.z = orient_z
        orientation_constraint.orientation.w = orient_w
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0

        #Aggiungo i vincoli
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def send_goal(self):
        """Invia un goal al server MoveGroup utilizzando la posizione target ricevuta."""

        self.robot_is_ready = False #il robot non è piu pronto, ma busy
        self.status_publisher.publish(Bool(data=self.robot_is_ready))


        if not self.target_pose:
            self.get_logger().warn('No target pose received yet. Waiting...')
            return

        self.get_logger().info('Waiting for move_group action server...')
        self.action_client.wait_for_server()

        # Creazione del messaggio di goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'

        # Configurazione della pose target direttamente dalla posa trasformata
        target_pose = self.target_pose
        target_pose.header.frame_id = BASE_RF  # Assicurarsi che il frame sia corretto

        # Orientamento del end-effector
        quaternion = self.obtain_quaternion(
                target_pose.header.frame_id, 
                90,
                target_pose.pose.position.x,
                target_pose.pose.position.y, 
                target_pose.pose.position.z )
                
        # Crea vincoli di movimento 
        constraints = self.make_constraints(
                self.target_pose.pose.position.x,
                self.target_pose.pose.position.y,
                self.target_pose.pose.position.z,
                quaternion.x, quaternion.y, quaternion.z, quaternion.w
                )
        
        # Imposta i vincoli come obiettivo
        goal_msg.request.goal_constraints.append(constraints)

        # Invio del goal
        self.get_logger().info('Sending goal to move_group...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        """Gestisce la risposta del server MoveGroup al goal inviato."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected by move_group.')
                return
            self.get_logger().info('Goal accepted by move_group. Waiting for result...')
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f"Goal response error: {str(e)}")

    def result_callback(self, future):
        """Gestisce il risultato del goal inviato."""
        try:
            result = future.result().result
            if result.error_code.val == 1:  # 1 indica SUCCESSO nella MoveIt error codes
                self.get_logger().info('Goal reached successfully!')
                self.get_logger().info('Need to grasp the apple')

                # FACCIO IL GRASPING
                ####### DA METTERE A GRASPING AVVENUTO (PER ORA METTO QUI)
                self.go_to_basket() # Mando il robot al cesto
            else:
                self.get_logger().error(f'MoveGroup failed with error code: {result.error_code.val}')
               
                #il robot NON riesce ad andare in quella posizione quindi è di nuovo libero
                #attendo prossime coordinate
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))

        except Exception as e:
            self.get_logger().error(f"Result callback error: {str(e)}")
            
            #il robot NON riesce ad andare in quella posizione quindi è di nuovo libero
            #attendo nuove coordinate
            self.robot_is_ready = True
            self.status_publisher.publish(Bool(data=self.robot_is_ready))

    
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
                to_basket_orientation_x,
                to_basket_orientation_y,
                to_basket_orientation_z,
                to_basket_orientation_w
            )

        basket_goal.request.goal_constraints.append(constraints)

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

                #DA METTERE DOPO IL RILASCIO DELLA MELA (PER ORA METTO QUI)
                #il robot ha raggiunto l'obiettivo ed è libero
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))      

            else:
                self.get_logger().error(f'Basket goal failed with error code: {result.error_code.val}')
                self.get_logger().error(f'Robot grasped the apple, but can''t go to the basket: {result.error_code.val}')

                ## LA MELA DEVE ESSERE RILASCIATA
                # rimetto il robot libero
                self.robot_is_ready = True
                self.status_publisher.publish(Bool(data=self.robot_is_ready))

        except Exception as e:
            self.get_logger().error(f"Error receiving basket goal result: {e}")
            self.get_logger().error(f'Robot grasped the apple, but can''t go to the basket: {result.error_code.val}')
            
            ## LA MELA DEVE ESSERE RILASCIATA
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