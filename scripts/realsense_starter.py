#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node as LaunchNode


class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_starter_node')
        self.get_logger().info("Configurazione RealSense Camera...")

        # Parametri per la fotocamera
        self.parameters = {
            'depth_width': 848,
            'depth_height': 480,
            'depth_fps': 15.0,
            'color_width': 848,
            'color_height': 480,
            'color_fps': 15.0,
            'align_depth.enable': True,
            'pointcloud.enable': True,
            'depth_module.enable_auto_exposure': True,
            'depth_module.hdr_enabled': True,
            'enable_sync': True,
            'enable_rgbd': True,
            'enable_gyro': True,
            'enable_accel': True, 
            'gyro_fps': 15.0,
            'accel_fps': 15.0, 
            'unite_imu_method': 2,
            'tf_publish_rate': 15.0,
            'decimation_filter.enable': True,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True, 
            'disparity_filter.enable': True,
            'hole_filling_filter.enable': True
        }

        # Path al file launch di realsense-ros
        realsense_launch_file = os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py'
        )

        # Avvia il launch file con parametri personalizzati
        self.get_logger().info("Eseguendo launch file con parametri personalizzati.")
        self.start_realsense_launch(realsense_launch_file, self.parameters)

    def start_realsense_launch(self, launch_file, params):
        from launch import LaunchService
        from launch.substitutions import LaunchConfiguration
        from launch.actions import DeclareLaunchArgument
        from launch_ros.descriptions import ParameterValue

        # Converti i parametri in un formato adatto
        launch_args = [
            DeclareLaunchArgument(
                key,
                default_value=str(value)
            )
            for key, value in params.items()
        ]

        # Includi il launch file con i parametri
        launch_description = LaunchDescription(launch_args)
        launch_description.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={key: str(value) for key, value in params.items()}.items()
            )
        )

        # Avvia il servizio di launch
        launch_service = LaunchService()
        launch_service.include_launch_description(launch_description)
        launch_service.run()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
