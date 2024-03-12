import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def get_absoulute_path(name, path):
    package_path = get_package_share_directory(name)
    return os.path.join(package_path, path)

def load_file(package_name, file_path):
    try:
        with open(get_absoulute_path(package_name, file_path), "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    try:
        with open(get_absoulute_path(package_name, file_path), "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):

    group_competitor = Node(
        package="group2",
        executable="competitor",
        output="screen",
    )

    nodes_to_start = [
        group_competitor
    ]

    return nodes_to_start

def generate_launch_description():

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
