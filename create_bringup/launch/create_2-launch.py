import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='config',
            default_value= os.path.join(get_package_share_directory('create_bringup'),"config", "default.yaml")
        ),
        launch.actions.DeclareLaunchArgument(
            name='desc',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='create_driver',
            executable='create_driver',
            name='create_driver',
            output='screen',
            parameters=[
                {
                    'robot_model': 'CREATE_2'
                },
                launch.substitutions.LaunchConfiguration('config')
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
