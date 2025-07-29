import os
import yaml
from ament_index_python.packages import get_package_share_path, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    robotic_config_path = get_package_share_path('robotic_config')
    default_rviz_config_path = robotic_config_path / 'rviz/view.rviz'

    robot_ip_arg = DeclareLaunchArgument(name='robot_ip')
    robot_ip = LaunchConfiguration('robot_ip')

    has_gripper_arg = DeclareLaunchArgument(name='has_gripper', default_value='false', choices=['true', 'false'])
    has_gripper = LaunchConfiguration('has_gripper')

    robotic_interface_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lebai_driver'),
                'launch',
                'robot_interface.launch.py'
            ])
        ]),
        launch_arguments={
            'has_gripper': has_gripper,
            'robot_ip': robot_ip
        }.items()
    )

    robotic_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("robotic_config"),
            "urdf",
            "robotic_with_zed.xacro",
        )
    )
    robotic_description = {"robot_description": robotic_description_config.toxml()}

    robotic_description_semantic_config = load_file(
        "robotic_config", "config/robotic.srdf"
    )
    robotic_description_semantic = {
        "robot_description_semantic": robotic_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "robotic_config", "config/kinematics.yaml"
    )

    ompl_planning_pipeline_config = {
        "grabbing_node": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        "robotic_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["grabbing_node"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "robotic_config", "config/controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    run_move_group_node = Node(
        package="target_grabbing",
        executable="grabbing_node",
        output="screen",
        parameters=[
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            robotic_description,
            robotic_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # rviz_config_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
    #                                  description='Absolute path to rviz config file')

    # joint_state_publisher_arg = DeclareLaunchArgument(name='joint_state_publisher', default_value='true', choices=['true', 'false'],
    #                                 description='Flag to enable joint_state_publisher_gui')

    # model_file= LaunchConfiguration('model')
    joint_state_publisher_condition = LaunchConfiguration('joint_state_publisher')
    # rviz_condition = LaunchConfiguration('rviz')

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    robotic_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robotic_description],
    )

    # joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     parameters=[{'robot_description': robot_description}],
    #     output='screen',
    #     condition=IfCondition(joint_state_publisher_condition),
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(default_rviz_config_path)],
    )

    return LaunchDescription([
        robot_ip_arg,
        has_gripper_arg,
        robotic_interface_node,
        # rviz_config_arg,
        static_tf,
        robotic_state_publisher,
        run_move_group_node,
        rviz_node,
    ])