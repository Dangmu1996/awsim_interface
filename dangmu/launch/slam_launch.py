import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package = "joy",
            executable ="joy_node",
            name= 'joy_node'
        ),
        launch_ros.actions.Node(
            package ="joystick_control",
            executable = "joystick_interface",
            name='joystick_interface'
        ),
        launch_ros.actions.Node(
            package ="dangmu",
            executable = "convert_imu",
            name='convert_imu'
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('lidarslam'), 'launch', 'lidarslam.launch.py'])
        )
  ])