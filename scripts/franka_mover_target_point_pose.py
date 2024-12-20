#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup



class MoveFR3(Node):
    def __init__(self):
        super().__init__('move_fr3_node')

        # Action client per MoveGroup
        self.action_client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self):
        self.get_logger().info('Waiting for move_group action server...')
        self.action_client.wait_for_server()

        # Creazione del messaggio di goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'

        # Configurazione della pose target
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'fr3_link0'
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.1
        target_pose.pose.position.z = 0.8
        #Quaternione della posa   
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        # Vincoli di posizione
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id
        position_constraint.link_name = 'fr3_link8'  # Link finale del manipolatore
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        )
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        position_constraint.weight = 0.0

        # Vincoli di orientamento
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id
        orientation_constraint.link_name = 'fr3_link8'
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 2.0

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
        result = future.result().result  # Ottenere il risultato dell'azione
        if result.error_code.val == 1:  # 1 indica SUCCESSO nella MoveIt error codes
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'MoveGroup failed with error code: {result.error_code.val}')



def main(args=None):
    rclpy.init(args=args)
    node = MoveFR3()
    node.send_goal()
    rclpy.spin_once(node)

    node.get_logger().info("Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
