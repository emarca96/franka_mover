#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Action client per il gripper
        self.action_client = ActionClient(self, GripperCommand, '/fr3_gripper/gripper_action')

        # Verifica della disponibilità del server d'azione
        self.get_logger().info("Waiting for gripper action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
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


def main(args=None):
    try:
        rclpy.init(args=args)

        node = GripperController()
        if node.action_client is not None:
            node.send_gripper_command(position=0.01, max_effort=10.0)
            rclpy.spin(node)
        else:
            node.get_logger().error("Exiting due to unavailability of the action server.")
    except KeyboardInterrupt:
        if 'node' in locals():
            node.get_logger().info("Shutting down...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
