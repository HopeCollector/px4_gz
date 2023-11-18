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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = FindPackageShare("px4_gz_bringup")
    pkg_project_gazebo = FindPackageShare("px4_gz_gazebo")
    pkg_project_description = FindPackageShare("px4_gz_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # start px4 controller
    cmd_px4 = ExecuteProcess(
        cmd=[
            "bash",
            PathJoinSubstitution([pkg_project_bringup, "config", "launch_px4.sh"]),
        ],
        output="screen",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": PythonExpression(
                [
                    " '-r -s --headless-rendering ' + '",  # running in headless mode
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
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "x500_lidar/link",
            "--child-frame-id", "x500_lidar/link/gpu_lidar",
        ],
        output="screen",
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [pkg_project_bringup, "config", "px4_gz_bridge.yaml"]
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    # container for mesh loader
    container = ComposableNodeContainer(
        name="px4_gz",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="px4_gz_application",
                plugin="px4_gz::rivz_mesh_loader",
                name="rivz_mesh_loader",
            )
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,  # simulator
            tf_static_node,  # lidar frame id translate
            cmd_px4,  # px4 controller
            bridge,  # gz <-> ros2 bridge
            container,
        ]
    )
