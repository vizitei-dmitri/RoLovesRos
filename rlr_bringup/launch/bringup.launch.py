from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #  parametrs

    # 2. Запуск камеры (из пакета rlr_camera)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rlr_camera'),
            'launch',
            'camera.launch.py'  
        ]),
    )

    # 3. Запуск AprilTags (из пакета rlr_april_tags)
    april_tags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('rlr_april_tags'),
            'launch',
            'detect_apriltags.launch.py'  
        ]),
    )

    return LaunchDescription([

        camera_launch,
        april_tags_launch
    ])
