import os
import sys

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('create_bringup'),
        'config',
        'xbox360.yaml'
    )
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='joy_dev',
            default_value='/dev/input/js0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='joy_config',
            default_value='xbox360'
        ),
        launch.actions.DeclareLaunchArgument(
            name='teleop_config',
            default_value=launch.substitutions.LaunchConfiguration(
                'joy_config')
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {
                    'dev': launch.substitutions.LaunchConfiguration('joy_dev')
                },
                {
                    'deadzone': 0.2
                },
                {
                    'autorepeat_rate': 20.0
                }
            ]
        ),
        launch_ros.actions.Node(
            package='joy_teleop',
            executable='joy_teleop',
            name='joy_teleop',
            parameters = [config]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
