# Copyright 2025 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace, SetParameter

from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    # Package Directories
    pkg_duatic_control = FindPackageShare("duatic_control")
    pkg_duatic_simulation = FindPackageShare("duatic_simulation")

    # Process URDF file
    doc = xacro.parse(open(LaunchConfiguration("urdf_file_path").perform(context)))
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    xacro.process_doc(
        doc,
        mappings={
            "namespace": LaunchConfiguration("namespace").perform(context),
            "mode": "sim",
            "dof": LaunchConfiguration("dof").perform(context),
            "covers": LaunchConfiguration("covers").perform(context),
            "version": LaunchConfiguration("version").perform(context),
            "tf_prefix": tf_prefix + "/" if tf_prefix else "",
        },
    )

    # Process ros2_control_params file
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    if tf_prefix != "":
        prefix = tf_prefix + "/"
        suffix = "_" + tf_prefix
    else:
        prefix = ""
        suffix = ""

    controllers_params = ReplaceString(
        source_file=LaunchConfiguration("ros2_control_params_arm"),
        replacements={"<prefix>": prefix, "<suffix>": suffix},
    )

    group_action = GroupAction(
        actions=[
            # Push Namespace, if the component is started as a standalone component
            PushRosNamespace(
                LaunchConfiguration("namespace"),
                condition=UnlessCondition(LaunchConfiguration("start_as_subcomponent")),
            ),
            # Set use_sim_time parameter
            SetParameter(name="use_sim_time", value=True),
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": doc.toxml()}],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                condition=UnlessCondition(LaunchConfiguration("start_as_subcomponent")),
            ),
            # Spawn in Simulation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_duatic_simulation, "launch", "spawn.launch.py"])
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "x": LaunchConfiguration("initial_pose_x"),
                    "y": LaunchConfiguration("initial_pose_y"),
                    "z": LaunchConfiguration("initial_pose_z"),
                    "yaw": LaunchConfiguration("initial_pose_yaw"),
                }.items(),
                condition=UnlessCondition(LaunchConfiguration("start_as_subcomponent")),
            ),
            # Start Controllers
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_duatic_control, "launch", "control.launch.py"])
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "config_path": controllers_params,
                }.items(),
            ),
            # Emergency Stop
            Node(
                package="dynaarm_extensions",
                executable="e_stop_node",
                name="e_stop_node",
                output="screen",
                parameters=[{"emergency_stop_button": 9}],  # Change button index here
                condition=UnlessCondition(LaunchConfiguration("start_as_subcomponent")),
            ),
        ]
    )

    return [group_action]


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            name="dof",
            choices=["1", "2", "3", "4", "5", "6"],
            default_value="6",
            description="Select the desired degrees of freedom (dof)",
        ),
        DeclareLaunchArgument(
            name="covers",
            default_value="False",
            description="Show or hide the covers of the robot",
        ),
        DeclareLaunchArgument(
            name="version",
            default_value="baracuda12",
            choices=["arowana4", "baracuda12"],
            description="Select the desired version of robot ",
        ),
        DeclareLaunchArgument(name="namespace", default_value="", description="Robot namespace"),
        DeclareLaunchArgument(
            name="urdf_file_path",
            default_value=get_package_share_directory("dynaarm_description")
            + "/urdf/dynaarm_standalone.urdf.xacro",
            description="Path to the robot URDF file",
        ),
        DeclareLaunchArgument(
            "ros2_control_params_arm",
            default_value=get_package_share_directory("dynaarm_bringup")
            + "/config/controllers_sim.yaml",
            description="Path to the controllers config file",
        ),
        DeclareLaunchArgument(
            "start_as_subcomponent",
            default_value="false",
            description="Whether the dynaarm is started as a subcomponent",
        ),
        DeclareLaunchArgument(
            "initial_pose_x", default_value="0.0", description="Spawn position in axis x."
        ),
        DeclareLaunchArgument(
            "initial_pose_y", default_value="0.0", description="Spawn position in axis y."
        ),
        DeclareLaunchArgument(
            "initial_pose_z", default_value="0.0", description="Spawn position in axis z."
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw", default_value="0.0", description="Spawn rotation around axis z."
        ),
        DeclareLaunchArgument("tf_prefix", default_value="", description="Arm identifier"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
