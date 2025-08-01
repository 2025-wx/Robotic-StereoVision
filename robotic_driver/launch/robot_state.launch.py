from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    has_gripper_arg = DeclareLaunchArgument(name='has_gripper', default_value='false', choices=['true', 'false'],
                                            description='mount gripper on the end')
    # launch.actions.substitutions.
    has_gripper = LaunchConfiguration('has_gripper')

    robotic_joint_names = PathJoinSubstitution([
        FindPackageShare('robotic_driver'),
        'config',
        'joint_names_robotic.yaml'
    ])
    gripper_joint_names = PathJoinSubstitution([
        FindPackageShare('robotic_driver'),
        'config',
        'joint_names_gripper.yaml'
    ])
    robot_ip_arg = DeclareLaunchArgument(name='robot_ip', default_value=str(),
                                         description='Robot ip address to connect.')

    robot_ip = LaunchConfiguration('robot_ip')
    robot_state_node = Node(
        package='robotic_driver',
        executable='robot_state',
        parameters=[robotic_joint_names, gripper_joint_names,
                    {"robot_ip_address": robot_ip},{"has_gripper": has_gripper}],
        output='screen',
    )

    return LaunchDescription([
        has_gripper_arg,
        robot_ip_arg,
        robot_state_node,
    ])
