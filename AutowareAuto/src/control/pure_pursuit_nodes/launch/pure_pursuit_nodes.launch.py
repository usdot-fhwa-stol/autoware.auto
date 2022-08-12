from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():

    param_file_path = os.path.join(
        get_package_share_directory('pure_pursuit_nodes'), 'param/pure_pursuit.param.yaml')

    return LaunchDescription([
        Node(
            package='pure_pursuit_nodes',
            namespace='',
            executable='pure_pursuit_node_exe',
            name='pure_pursuit_node',
            parameters=[ param_file_path ]
        )
    ])