"""
Lane Tracker - Akadálykerülő és cliff-detektáló jármű szimulációja
==================================================================
Ez a launch fájl indítja el a komplett szimulációs rendszert:
1. Gazebo szimulátort az SDF modellel
2. ROS-Gazebo bridge-et a topic kommunikációhoz
3. ObjectAvoider csomópontot az autonóm vezetéshez

Az akadálykerülés és cliff-detekció a konszolon nyomott üzenetek formájában jelenik meg.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    ROS2 launch leírás generálása.
    
    A szekvencia:
    1. Gazebo szimulátort indít az environment_car.sdf modellel
    2. ROS-Gazebo bridge-et indít a topic-ok átviteléhez
    3. ObjectAvoider csomópontot indít az autonóm vezetéshez
    """
    
    # Csomag és fájlok helyzete
    pkg_lane_tracker = get_package_share_directory('lane_tracker')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    sdf_file = os.path.join(pkg_lane_tracker, 'simulation', 'environment_car.sdf')

    # --- 1. GAZEBO SZIMULÁTOR INDÍTÁSA ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # --- 2. ROS-GAZEBO BRIDGE ---
    # A bridge biztosítja a kommunikációt ROS2 topic-ok és Gazebo között
    # Topic mapping:
    #   - /cmd_vel:      Jármű sebességparancs (ROS2 -> Gazebo)
    #   - /scan:         Elülső lézer szenzor (Gazebo -> ROS2)
    #   - /odom:         Odometria - jármű pozíciója és orientációja (Gazebo -> ROS2)
    #   - /cliff_left:   Bal cliff szenzor (Gazebo -> ROS2)
    #   - /cliff_right:  Jobb cliff szenzor (Gazebo -> ROS2)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Sebesség parancs küldetése a járműnek
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # Szenzor adatok fogadása
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/cliff_left@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/cliff_right@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'
        ],
        output='screen'  # Konzol kimenet engedélyezése debuggoláshoz
    )

    # --- 3. OBJECTAVOIDER CSOMÓPONT ---
    # Az autonóm vezetési logika
    # Ez a csomópont:
    #   - Fogadja az szenzor adatokat (/scan, /cliff_left, /cliff_right, /odom)
    #   - Feldolgozza őket az akadálykerülés és cliff-detekció logikájával
    #   - Konzolra nyomtat üzeneteket az akadályokról és a kitérésekről
    #   - Parancsokat küld a járműnek (/cmd_vel)
    object_avoider = Node(
        package='lane_tracker',
        executable='object_avoider',
        name='object_avoider',
        output='screen',  # Konzol kimenet - itt jelennek meg az akadálydetektor üzenetei!
        parameters=[
            {'drive_speed': 1.0},  # Jármű előre haladási sebessége (m/s)
            {'kp': 2.0}            # PID proporcionális erősítés (szögsebesség szabályozáshoz)
        ]
    )

    # Összes csomópont és szimulátort indítása
    return LaunchDescription([gz_sim, bridge, object_avoider])