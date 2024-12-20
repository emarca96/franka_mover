#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Variabili globali per i reference frame 
START_RF = "camera_link"  # Nome RF centrale videocamera 
TARGET_RF = "fr3_link0"  # Nome RF base del robot
DEPTH_VIEW_RF = "camera_depth_optical_frame"  # Nome RF riferimento per le depth images
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

        # Sottoscrizione al topic /apple_coordinates
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apple_coordinates',
            self.apple_callback,
            10  # QoS depth
        )
        self.get_logger().info('Subscribed to /apple_coordinates')

        # Variabile per memorizzare la posizione target
        self.target_pose = None

        # Pubblicazione del reference frame statico
        self.broadcast_static_transform()

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
        """Callback per il topic /apple_coordinates."""
        self.get_logger().info(f"Received apple coordinates: x={msg.pose.position.x}, "
                               f"y={msg.pose.position.y}, z={msg.pose.position.z}")
        self.target_pose = msg
        self.send_goal()

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

        # Configurazione della pose target
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'fr3_link0'
        target_pose.pose.position.x = self.target_pose.pose.position.x
        target_pose.pose.position.y = self.target_pose.pose.position.y
        target_pose.pose.position.z = self.target_pose.pose.position.z

        # Copia l'orientamento dal messaggio ricevuto
        target_pose.pose.orientation.x = self.target_pose.pose.orientation.x
        target_pose.pose.orientation.y = self.target_pose.pose.orientation.y
        target_pose.pose.orientation.z = self.target_pose.pose.orientation.z
        target_pose.pose.orientation.w = self.target_pose.pose.orientation.w

        # Vincoli di posizione
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = 'fr3_link8'  # Link finale del manipolatore
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        )
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        position_constraint.weight = 1.0

        # Vincoli di orientamento
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = 'fr3_link8'
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