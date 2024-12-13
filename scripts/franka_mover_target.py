#!/usr/bin/env python3

#import sys
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class FrankaMoverTarget(Node):
    def __init__(self):
        super().__init__('franka_mover_target')

        # Publisher per la traiettoria dei giunti
        self.arm_publisher = self.create_publisher(JointTrajectory, '/franka/follow_joint_trajectory', 10)

        # Publisher per il controllo del gripper
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/franka/gripper_command', 10)

    def send_joint_trajectory(self, joint_positions, time_from_start=2.0):
        """
        Invia una traiettoria ai giunti del braccio.
        :param joint_positions: Lista delle posizioni target per i giunti.
        :param time_from_start: Tempo per completare il movimento in secondi.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
        trajectory.points = [point]

        self.get_logger().info(f"Publishing joint trajectory: {joint_positions}")
        self.arm_publisher.publish(trajectory)

    def control_gripper(self, width: float, time_from_start=1.0):
        """
        Controlla l'apertura del gripper.
        :param width: Ampiezza target del gripper in metri.
        :param time_from_start: Tempo per completare il movimento in secondi.
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = ['fr3_finger_joint1', 'fr3_finger_joint2']

        point = JointTrajectoryPoint()
        point.positions = [width, width]  # Posizioni simmetriche per il gripper
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)
        trajectory.points = [point]

        self.get_logger().info(f"Publishing gripper command with width: {width}")
        self.gripper_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)

    # Inizializza il nodo
    node = FrankaMoverTarget()

    # Definisci le posizioni target per i giunti del braccio
    joint_positions = [0.5, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5]  # Modifica queste posizioni secondo necessità
    node.send_joint_trajectory(joint_positions, time_from_start=2.0)

    # Controlla il gripper (ad esempio, apertura a 0.04 metri)
    node.control_gripper(0.04, time_from_start=1.0)

    # Chiudi il gripper (ad esempio, 0.0 metri)
    node.control_gripper(0.0, time_from_start=1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


#avviare con robot reale: python3 franka_mover_targer.py
#avviare con UI: python3 franka_mover_target.py --fake
