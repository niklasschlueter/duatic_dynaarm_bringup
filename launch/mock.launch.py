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

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    # Package Directories
    pkg_duatic_control = FindPackageShare("duatic_control")

    # Process URDF file
    doc = xacro.parse(open(LaunchConfiguration("urdf_file").perform(context)))
    xacro.process_doc(
        doc,
        mappings={
            "namespace": LaunchConfiguration("namespace").perform(context),
            "mode": "mock",
            "dof": LaunchConfiguration("dof").perform(context),
            "covers": LaunchConfiguration("covers").perform(context),
            "version": LaunchConfiguration("version").perform(context),
            "tf_prefix": LaunchConfiguration("tf_prefix").perform(context) + "/",
        },
    )

    # Process SRDF file
    srdf_doc = xacro.parse(open(LaunchConfiguration("srdf_file").perform(context)))
    xacro.process_doc(
        srdf_doc,
        mappings={
            "dof": LaunchConfiguration("dof").perform(context),
            "covers": LaunchConfiguration("covers").perform(context),
            "version": LaunchConfiguration("version").perform(context),
            "mode": "mock",
        },
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": doc.toxml()}],
        namespace=LaunchConfiguration("namespace"),
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[{"update_rate": 1000}],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Ros2 Controllers
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    if tf_prefix != "":
        prefix = tf_prefix + "/"
        suffix = "_" + tf_prefix
    else:
        prefix = ""
        suffix = ""

    srdf_str = srdf_doc.toxml().replace("\n", "\\n").replace('"', '\\"')
    controllers_params = ReplaceString(
        source_file=LaunchConfiguration("controllers_config"),
        replacements={
            "<prefix>": prefix,
            "<suffix>": suffix,
            "<srdf>": f'"{srdf_str}"',
        },
    )

    start_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_duatic_control, "launch", "control.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "config_path": controllers_params,
        }.items(),
    )

    # Emergency Stop
    e_stop_node = Node(
        package="dynaarm_extensions",
        executable="e_stop_node",
        namespace=LaunchConfiguration("namespace"),
        name="e_stop_node",
        output="screen",
        parameters=[{"emergency_stop_button": 9}],  # Change button index here
    )

    # Collect nodes to start
    nodes_to_start = [
        start_controllers,
    ]

    # Only start these nodes if not started as subcomponent
    if LaunchConfiguration("start_as_subcomponent").perform(context).lower() == "false":
        nodes_to_start.append(robot_state_publisher)
        nodes_to_start.append(controller_manager)
        nodes_to_start.append(e_stop_node)

    return nodes_to_start


def generate_launch_description():

    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="gui",
            default_value="True",
            choices=["True", "False"],
            description="Flag to enable joint_state_publisher_gui",
        ),
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
        DeclareLaunchArgument(
            name="urdf_file",
            default_value=get_package_share_directory("dynaarm_description")
            + "/urdf/dynaarm_standalone.urdf.xacro",
        ),
        DeclareLaunchArgument(
            name="srdf_file",
            default_value=get_package_share_directory("dynaarm_description")
            + "/config/dynaarm.srdf",
            description="Path to the robot SRDF file",
        ),
        DeclareLaunchArgument(
            "start_as_subcomponent",
            default_value="false",
            description="Whether the platform is started as a subcomponent",
        ),
        DeclareLaunchArgument(
            "controllers_config",
            default_value=get_package_share_directory("dynaarm_bringup")
            + "/config/controllers.yaml",
            description="Path to the controllers config file",
        ),
        DeclareLaunchArgument(
            name="namespace",
            default_value="dynaarm1",
        ),
        DeclareLaunchArgument("tf_prefix", default_value="arm_1", description="Arm identifier"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
