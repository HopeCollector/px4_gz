# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = get_package_share_directory("px4_gz_bringup")
    pkg_project_gazebo = get_package_share_directory("px4_gz_gazebo")
    pkg_project_description = get_package_share_directory("px4_gz_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # start px4 controller
    cmd_px4 = ExecuteProcess(
        cmd=[
            "bash",
            os.path.join(pkg_project_bringup, "config", "launch_px4.sh"),
        ],
        output="screen",
    )

    # Load the SDF file from "description" package
    sdf_file = os.path.join(
        pkg_project_description, "models", "x500_lidar_desc", "model.sdf"
    )
    with open(sdf_file, "r") as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(
                [
                    " '-r -s --headless-rendering ' + '", # running in headless mode
                    # " '-r ' + '", # running in gui mode
                    PathJoinSubstitution([pkg_project_gazebo, "worlds", "px4_gz.sdf"]),
                    "'",
                ]
            )
        }.items(),
    )

    # tf static transform between lidar_link and gpu_lidar
    tf_static_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        exec_name="tf_static_lidar",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "x500_lidar/lidar_3d_v1/link",
            "x500_lidar/lidar_3d_v1/link/gpu_lidar",
        ],
        output="screen",
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    pkg_project_bringup, "config", "px4_gz_bridge.yaml"
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim, # simulator
            cmd_px4,# px4 controller
            bridge, # gz <-> ros2 bridge
            tf_static_node, # lidar frame id translate
        ]
    )
