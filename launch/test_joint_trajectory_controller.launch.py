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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # controller type argument
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="hebi_arm_controller",
        choices=["hebi_arm_controller", "hebi_arm_with_gripper_controller"],
        description="Type of controller to test (determines which config file to use).",
    )

    # config file argument
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="test_hebi_arm_controller.yaml",
        description="Name of the config file to use for the test. Overrides controller_type if specified.",
    )

    controller_type = LaunchConfiguration("controller_type")
    config_file = LaunchConfiguration("config_file")

    # Determine config file based on controller_type if config_file is at default value
    resolved_config_file = PythonExpression([
        "'test_hebi_arm_with_gripper_controller.yaml' if '",
        controller_type,
        "' == 'hebi_arm_with_gripper_controller' and '",
        config_file,
        "' == 'test_hebi_arm_controller.yaml' else '",
        config_file,
        "'"
    ])

    position_goals = PathJoinSubstitution(
        [FindPackageShare("hebi_bringup"), "config", resolved_config_file]
    )

    return LaunchDescription(
        [
            controller_type_arg,
            config_file_arg,
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_joint_trajectory_controller",
                parameters=[position_goals],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            )
        ]
    )
