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
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_bringup = FindPackageShare("px4_gz_bringup")
    pkg_project_gazebo = FindPackageShare("px4_gz_gazebo")
    pkg_project_description = FindPackageShare("px4_gz_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # start px4 controller
    px4_controller = ExecuteProcess(
        cmd=[
            "bash",
            PathJoinSubstitution([pkg_project_bringup, "config", "launch_px4.sh"]),
        ],
        name="px4_controller",
        output="screen",
    )

    # microdds
    px4_bridge = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="px4_bridge",
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

    # Bridge ROS topics and Gazebo messages for establishing communication
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
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
        name="px4_gz_app_container",
        namespace="px4_gz",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="px4_gz_application",
                plugin="px4_gz::visualization_helper",
                name="visualization_helper",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_file_path": PathJoinSubstitution(
                            [pkg_project_gazebo, "worlds", "px4_gz.yaml"]
                        ),
                        "odom_frame_id": "odom_ned",
                    }
                ],
                remappings=[
                    ("sub/odom", "odom"),
                ],
            ),
            ComposableNode(
                package="px4_gz_application",
                plugin="px4_gz::odometry",
                name="odometry",
                parameters=[
                    {
                        "use_sim_time": True,
                        "odom_frame_id": "odom",
                        "base_link_frame_id": "x500_lidar/base_link",
                    }
                ],
                remappings=[
                    ("pub/odom", "odom"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gz_sim,  # simulator
            gz_bridge,  # gz <-> ros2 bridge
            px4_controller,  # px4 controller
            px4_bridge,  # px4 <-> ros2 bridge
            PushRosNamespace(
                "px4_gz"
            ),  # push namespace to all nodes in this launch file
            container,  # visualization helper
        ]
    )
