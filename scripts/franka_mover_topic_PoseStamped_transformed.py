#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped,TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped


# Variabili globali per i reference frame 
START_RF = "camera_link"  # Nome RF centrale videocamera 
TARGET_RF = "fr3_link0"  # Nome RF base del robot
# DEPTH_VIEW_RF = "camera_depth_optical_frame"  # Nome RF riferimento per le depth images
translation_x = 0.0  # Traslazione in x da START_RF a TARGET_RF
translation_y = 0.0  # Traslazione in y da START_RF a TARGET_RF
translation_z = 0.0  # Traslazione in z da START_RF a TARGET_RF
rotation_x = 0.0  # Rotazione x da START_RF a TARGET_RF
rotation_y = 0.0  # Rotazione y da START_RF a TARGET_RF
rotation_z = 0.0  # Rotazione z da START_RF a TARGET_RF
rotation_w = 1.0  # Rotazione w da START_RF a TARGET_RF
##########################################

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



    def send_goal(self):
        """Invia un goal al server MoveGroup utilizzando la posizione target ricevuta."""
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
        target_pose.header.frame_id = TARGET_RF  # Assicurarsi che il frame sia corretto

        # Vincoli di posizione
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = 'fr3_hand_tcp'  # Link finale del manipolatore
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        )
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        position_constraint.weight = 1.0

        # Vincoli di orientamento
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = 'fr3_hand_tcp'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # Configurazione dei vincoli
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

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
            else:
                self.get_logger().error(f'MoveGroup failed with error code: {result.error_code.val}')
        except Exception as e:
            self.get_logger().error(f"Result callback error: {str(e)}")


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