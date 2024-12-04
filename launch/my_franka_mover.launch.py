import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration arguments
    load_gripper = LaunchConfiguration('load_gripper', default='true')
    arm_id = LaunchConfiguration('arm_id', default='fr3')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.1')
    target_x = LaunchConfiguration('target_x', default='0.5')  # Default X coordinate
    target_y = LaunchConfiguration('target_y', default='0.0')  # Default Y coordinate
    target_z = LaunchConfiguration('target_z', default='0.2')  # Default Z coordinate
    
    # Path to the robot description and MoveIt configuration
    franka_description_pkg = get_package_share_directory('franka_description')  # Corretto
    moveit_config_pkg = get_package_share_directory('franka_fr3_moveit_config')  # Corretto
    gazebo_pkg = get_package_share_directory('franka_gazebo_bringup')  # Corretto
    
   # Gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(gazebo_pkg, 'launch', 'visualize_franka_robot.launch.py')
    ),
    launch_arguments={'gz_args': 'empty.sdf -r'}.items()  # puoi mantenere gli argomenti, se necessari
)

    
    # MoveIt launch file
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'load_gripper': load_gripper,
            'use_fake_hardware': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_description_xacro = os.path.join(franka_description_pkg, 'robots', 'fr3', 'fr3.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': os.path.join(robot_description_xacro)
        }]
    )
    
    # RViz Node
    rviz_config_file = os.path.join(franka_description_pkg, 'rviz', 'visualize_franka.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_file]
    )
    
    # Joint State Broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    # Joint Position Controller
    load_joint_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_position_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        # Declare arguments for target position
        DeclareLaunchArgument('target_x', default_value='0.5', description='Target X coordinate'),
        DeclareLaunchArgument('target_y', default_value='0.0', description='Target Y coordinate'),
        DeclareLaunchArgument('target_z', default_value='0.2', description='Target Z coordinate'),
        
        # Gazebo Simulation
        gazebo_launch,
        
        # MoveIt Configuration
        moveit_launch,
        
        # Robot Description Publisher
        robot_state_publisher,
        
        # RViz Visualization
        rviz_node,
        
        # Controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_position_controller]
            )
        ),
    ])
