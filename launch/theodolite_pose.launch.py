import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_folder = get_package_share_directory('theodolite_pose')
    config_file = os.path.join(share_folder, 'config', 'theodolite_pose.yaml')

    theodolite_pose_node = Node(
        package='theodolite_pose',
        executable='theodolite_pose_node',
        name='theodolite_pose_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        theodolite_pose_node
    ])