from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EqualsSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition


def generate_moveit_nodes(context, *args, **kwargs):
    # Resolve hebi_arm configuration during runtime
    hebi_arm = LaunchConfiguration("hebi_arm").perform(context)
    moveit_package = "hebi_" + hebi_arm.lower() + "_moveit_config"

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")

    description_filepath = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", "kits", "ros2_control", description_file]
    ).perform(context)
    
    moveit_config = MoveItConfigsBuilder(
        robot_name=hebi_arm,
        package_name=moveit_package
    ).robot_description(
        description_filepath
    ).to_moveit_configs()

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': LaunchConfiguration("use_sim_time")},
            ],
        )
    ]
 
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
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )
    )

    # Set default values for arguments
    default_arguments = []
    default_arguments.append(
        LogInfo(
            msg=PythonExpression(['"Using default description_file: ', LaunchConfiguration("hebi_arm"), '.urdf.xacro"']),
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('description_file'), "None")
            )
        )
    )
    default_arguments.append(
        SetLaunchConfiguration(
            name="description_file",
            value=PythonExpression(['"', LaunchConfiguration("hebi_arm"), '.urdf.xacro"']),
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('description_file'), "None")
            )
        )
    )

    return LaunchDescription(
        declared_arguments +
        default_arguments +
        [
            OpaqueFunction(function=generate_moveit_nodes)
        ]
    )
 