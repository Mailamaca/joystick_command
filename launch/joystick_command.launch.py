# https://roboticsbackend.com/ros2-yaml-params/

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

parameters_folder = 'config'
parameters_filename = 'defaults.yaml'


def generate_launch_description():

    ld = LaunchDescription()

    parameters_file = os.path.join(
        get_package_share_directory('joystick_command'),
        parameters_folder,
        parameters_filename
        )

    node=Node(
        package = 'joystick_command',
        name = 'custom_joystick_command', # same as first line of <parameters_filename>.yaml
        executable = 'joystick_command',
        output="screen",
        emulate_tty=True,
        parameters = [parameters_file]
    )
    
    ld.add_action(node)
    return ld