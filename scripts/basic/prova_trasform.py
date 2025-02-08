#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped 
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class PoseTransformer(Node):
    def __init__(self):
        super().__init__('pose_transformer_node')

        # Frame names
        self.START_RF = "camera_link"
        self.TARGET_RF = "fr3_link0"

        # Buffer e listener per TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pubblica la trasformazione statica
        self.broadcast_static_transform()

        # Sottoscrizione al topic /apple_coordinates_realsense
        self.subscription = self.create_subscription(
            PoseStamped,
            '/apple_coordinates_realsense',
            self.pose_callback,
            10  # QoS depth
        )
        self.get_logger().info('Subscribed to /apple_coordinates_realsense')

    def broadcast_static_transform(self):
        """Definisce e pubblica la trasformazione statica tra START_RF e TARGET_RF."""
        static_broadcaster = StaticTransformBroadcaster(self)

        # Configura il messaggio di trasformazione
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = self.TARGET_RF
        static_transform.child_frame_id = self.START_RF

        # Traslazione
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        # Rotazione
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Pubblica la trasformazione statica
        static_broadcaster.sendTransform(static_transform)
        self.get_logger().info(f"Static transform published: {self.START_RF} -> {self.TARGET_RF}")

    def pose_callback(self, msg: PoseStamped):
        """Callback per trasformare il messaggio PoseStamped ricevuto."""
        try:
            # Log della pose originale ricevuta
            self.get_logger().info(f"Received pose: position={msg.pose.position}, orientation={msg.pose.orientation}")

            # Trova la trasformazione dal frame sorgente al frame TARGET_RF
            transform = self.tf_buffer.lookup_transform(
                self.TARGET_RF,  # Frame di destinazione
                msg.header.frame_id,  # Frame sorgente
                rclpy.time.Time(),  # Tempo della trasformazione
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            # Trasforma il messaggio PoseStamped
            transformed_pose = do_transform_pose_stamped(msg, transform)

            # Log del risultato trasformato
            self.get_logger().info(f"Transformed pose: position={transformed_pose.pose.position}, orientation={transformed_pose.pose.orientation}")

        except Exception as e:
            self.get_logger().error(f"Error during pose transformation: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
