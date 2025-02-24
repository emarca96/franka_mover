import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

class CartesianPathExecutor(Node):
    def __init__(self):
        super().__init__('cartesian_path_executor')
        self.client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')

        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Waiting for /compute_cartesian_path service...')
        self.get_logger().info('Service /compute_cartesian_path is available.')

    def create_target_pose(self):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "fr3_link0"
        target_pose.pose.position.x = 0.3069
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.3
        target_pose.pose.orientation.x = 1.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0
        return target_pose

    def call_cartesian_path(self, target_pose):
        request = GetCartesianPath.Request()
        request.group_name = "fr3_arm"
        request.link_name = "fr3_hand_tcp"
        request.waypoints = [target_pose.pose]
        request.max_step = 0.01
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        return self.client.call_async(request)

    def execute_trajectory(self, trajectory):
        """ Invia la traiettoria al controller del robot """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal)
        self.get_logger().info('Trajectory sent to robot controller.')

    def execute_cartesian_path(self):
        target_pose = self.create_target_pose()#
        future = self.call_cartesian_path(target_pose)#
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.fraction > 0.0:
                self.get_logger().info(f"Successfully computed Cartesian path with {response.fraction * 100}% coverage.")
                self.execute_trajectory(response.solution)  # Invia la traiettoria calcolata
            else:
                self.get_logger().error("Failed to compute a full Cartesian path.")
        else:
            self.get_logger().error("Service call failed.")

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPathExecutor()
    node.execute_cartesian_path()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
