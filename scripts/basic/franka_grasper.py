#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp

## Grasp variabiles ##
gripper_open = 0.039
gripper_close = 0.00
max_effort = 100.0

class GraspController(Node):
    def __init__(self):
        super().__init__('gripper_controller')

        # Action client per il gripper
        self.action_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')

        # Verifica della disponibilità del server d'azione
        self.get_logger().info("Waiting for grasp action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Grasp action server not available. Exiting.")
            self.action_client = None  # Imposta a None per evitare ulteriori operazioni
            return

        self.get_logger().info("Grasp action server available.")

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
            self.get_logger().error(f"Result callback error: {e}")


def main(args=None):
    try:
        rclpy.init(args=args)

        node = GraspController()
        if node.action_client is not None:
            node.send_grasp_command(gripper_open, max_effort)
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
