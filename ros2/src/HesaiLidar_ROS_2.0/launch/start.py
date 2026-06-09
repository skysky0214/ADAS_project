from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config='/home/khj/25project_ws/ADAS_project/artifacts/rviz/adas_lidar_tracking.rviz'
    config_path='/home/khj/25project_ws/ADAS_project/ros2/src/HesaiLidar_ROS_2.0/config/config.yaml'
    return LaunchDescription([
        Node(
            namespace='hesai_ros_driver',
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            output='screen',
            parameters=[{'config_path': config_path}],
        ),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
