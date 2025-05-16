import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Load MoveIt config and add kinematics.yaml
    moveit_config = (
        MoveItConfigsBuilder("multi_arm", package_name="moveit_panda")
        .robot_description_semantic("config/multi_arm.srdf")
        .robot_description_kinematics("config/kinematics.yaml")  
        .to_moveit_configs()
    )

    # Generate robot_description from xacro
    xacro_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "multi_arm",
        "multi_arm.urdf.xacro"
    )
    robot_description = xacro.process_file(xacro_path, mappings={
        "arm_id": "fr3",
        "hand": "true",
        "ee_id": "franka_hand"
    }).toxml()

    moveit_config.robot_description = {"robot_description": robot_description}

    return LaunchDescription([
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),

        # Joint state GUI
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),

        # Move group node 
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[
                {"robot_description": robot_description},
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,  
                moveit_config.planning_scene_monitor,
                moveit_config.joint_limits,
                moveit_config.trajectory_execution,
                moveit_config.moveit_cpp,
                moveit_config.planning_pipelines,
                {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
                {"moveit_simple_controller_manager.fake_controllers": os.path.join(get_package_share_directory("moveit_panda"), "config", "ros2_controllers.yaml")},
                {"allow_trajectory_execution": True},
                {"fake_execution_type": "interpolate"},
                {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
                {"use_sim_time": True}
            ],
            output="screen"
        ),

        # Controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                os.path.join(
                    get_package_share_directory("moveit_panda"),
                    "config",
                    "ros2_controllers.yaml"
                ),
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,  
                {"robot_description": robot_description},
                {"use_sim_time": False},
            ]
        ),

        # Spawners
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda1_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda2_arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda1_gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda2_gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[
                {"robot_description": robot_description},
                moveit_config.robot_description_semantic,
                {"use_sim_time": True}
            ],
            arguments=["-d", os.path.join(
                get_package_share_directory("moveit_panda"),
                "config",
                "moveit.rviz"
            )]
        ),

        # Custom nodes
        Node(package="robot_bringup", executable="ee_marker"),
        Node(package="robot_bringup", executable="hand_tracking"),
        Node(package="robot_bringup", executable="teleop_moveit")
    ])
