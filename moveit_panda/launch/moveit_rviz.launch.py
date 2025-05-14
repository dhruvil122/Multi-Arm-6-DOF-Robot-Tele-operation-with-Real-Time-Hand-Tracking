from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

def generate_launch_description():
    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    ee_id = LaunchConfiguration("ee_id")

    declare_arm_id = DeclareLaunchArgument("arm_id", default_value="multi_arm")
    declare_load_gripper = DeclareLaunchArgument("load_gripper", default_value="true")
    declare_ee_id = DeclareLaunchArgument("ee_id", default_value="franka_hand")

    moveit_config = MoveItConfigsBuilder(
        robot_name="multi_arm",
        package_name="moveit_panda"
    ).to_moveit_configs()

    return LaunchDescription([
        declare_arm_id,
        declare_load_gripper,
        declare_ee_id,

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen"
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.planning_pipelines,
                moveit_config.trajectory_execution,
                moveit_config.moveit_cpp,
                moveit_config.planning_scene_monitor,
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
           
        )
    ])
