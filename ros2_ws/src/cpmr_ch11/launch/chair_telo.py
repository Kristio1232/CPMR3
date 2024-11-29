import os
import json
import sys
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    nchairs = 5
    for arg in sys.argv:
        if arg.startswith('nchairs:='):
           print(arg.split('chairs:=', 1)[1])
           nchairs = int(arg.split('chairs:=', 1)[1])
        elif ':=' in arg:
           print(f"Unknown argument in {arg}")
           sys.exit(0)
    print(f"Controlling {nchairs}")

    nodelist = []
    
    # Add teleop_twist_keyboard node for controlling the leader robot
    nodelist.append(
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            remappings=[
                ('/cmd_vel', '/chair_0/cmd_vel'),
            ],
        )
    )
    print(f"Teleop keyboard for leader chair added")

    # Add follower nodes
    for chair in range(1, nchairs):
        name = f'chair_{chair}'
        print(f"Processing {chair}")
        nodelist.append(
            Node(
                namespace = name,
                package='cpmr_ch11',
                executable='follow_chair',
                name='follow_chair',
                output='screen',
                parameters=[{'chair_name' : f"chair_{chair}", 'target_name' : f"chair_{chair-1}"}])
        )

    return LaunchDescription(nodelist)