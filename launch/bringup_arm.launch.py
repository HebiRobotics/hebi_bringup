# Copyright (c) 2023, HEBI Robotics Inc.
# Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, LogInfo, SetLaunchConfiguration
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "hebi_arm",
            description="Name of the robot to be used.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_package",
            default_value="hebi_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="None", # Default set later using the hebi arm name
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="hebi_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="None", # Default set later using the hebi arm name
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="hebi_arm_controller",
            choices=["hebi_arm_controller"],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to start RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_pkg",
            default_value="hebi_description",
            description="Package with the robot's config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file_path",
            default_value="None", # Default set later using the hebi arm name
            description="Path to the robot's config file relative to the package",
        )
    )

    # Set default values for arguments
    default_arguments = []
    default_arguments.append(
        LogInfo(
            msg=PythonExpression(['"Using default controllers_file: ', LaunchConfiguration("hebi_arm"), '_controllers.yaml"']),
            condition=LaunchConfigurationEquals("controllers_file", "None")
        )
    )
    default_arguments.append(
        SetLaunchConfiguration(
            name="controllers_file",
            value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '_controllers.yaml"']),
            condition=LaunchConfigurationEquals("controllers_file", "None")
        )
    )

    default_arguments.append(
        LogInfo(
            msg=PythonExpression(['"Using default description_file: ', LaunchConfiguration("hebi_arm"), '.urdf.xacro"']),
            condition=LaunchConfigurationEquals("description_file", "None")
        )
    )
    default_arguments.append(
        SetLaunchConfiguration(
            name="description_file",
            value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '.urdf.xacro"']),
            condition=LaunchConfigurationEquals("description_file", "None")
        )
    )

    default_arguments.append(
        LogInfo(
            msg=PythonExpression(['"Using default config_file_path: config/arms/', LaunchConfiguration("hebi_arm"), '.cfg.yaml"']),
            condition=LaunchConfigurationEquals("config_file_path", "None")
        )
    )
    default_arguments.append(
        SetLaunchConfiguration(
            name="config_file_path",
            value=PythonExpression(['"config/arms/', LaunchConfiguration("hebi_arm"), '.cfg.yaml"']),
            condition=LaunchConfigurationEquals("config_file_path", "None")
        )
    )


    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    controllers_package = LaunchConfiguration("controllers_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    config_pkg = LaunchConfiguration("config_pkg")
    config_file_path = LaunchConfiguration("config_file_path")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", "kits", "ros2_control", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "config_pkg:=",
            config_pkg,
            " ",
            "config_file:=",
            config_file_path
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(controllers_package), "config", controllers_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "hebi_arm.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, robot_controllers],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        condition=LaunchConfigurationEquals("use_rviz", "true"),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_names = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = []
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_controller_spawners[i - 1]
                    if i > 0
                    else joint_state_broadcaster_spawner,
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=inactive_robot_controller_spawners[i - 1]
                    if i > 0
                    else robot_controller_spawners[-1],
                    on_exit=[controller],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments +
        default_arguments +
        [
            control_node,
            robot_state_pub_node,
            rviz_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
