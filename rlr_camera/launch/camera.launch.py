from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #Take path to the yaml file 
	# change names, when Dima finished yaml
    config_path = os.path.join(
        get_package_share_directory('rlr_camera'),
        'config',
        'param.yaml'
    )

    camera_node=Node(
	package='v4l2_camera',
	executable='v4l2_camera_node',
	name='v4l2_camera',
	parameters=[config_path],
	output='screen'
    )

    return LaunchDescription([
	camera_node
    ])
