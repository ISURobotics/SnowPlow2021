import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([Node(package='tf2_ros', namespace='scan_to_map', executable='static_transform_publisher', arguments=["0","0","0","0","0","0","map","scan"])])
