from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rlr_camera"),
                        "launch",
                        "camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            {'image_source': "/dev/video0"},
        ],
    )

    april_tags_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rlr_april_tags"),
                        "launch",
                        "detect_apriltags.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
        ],
    )

    rlr_tf_analysis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rlr_tf_analysis"),
                        "launch",
                        "tf_analysis.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
        ],
    )


    rviz = Node(
            package='rviz2',
            executable='rviz2',
        )

    goal = Node(
            package='rlr_apriltag_nav2',
            executable='apriltag_to_nav2',
        )

    return LaunchDescription([
        camera_launch,
        april_tags_launch,
        #rlr_tf_analysis,
        goal,
        rviz
    ])
