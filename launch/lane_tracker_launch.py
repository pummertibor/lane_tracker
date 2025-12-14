import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_lane_tracker = get_package_share_directory('lane_tracker')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_lane_tracker, 'simulation', 'environment_car.sdf')

    # 1. Ignition Gazebo indítása
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # 2. Bridge (Híd) a ROS2 és Ignition között
    # Mapping: ROS Topic @ ROS Type @ Ignition Type
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
        ],
        output='screen'
    )

    # 3. A saját C++ vezérlő node
    object_avoider = Node(
        package='lane_tracker',
        executable='object_avoider',
        name='object_avoider',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        object_avoider
    ])