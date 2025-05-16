import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction

def generate_launch_description():

  
    moveit_config = (
        MoveItConfigsBuilder("multi_arm", package_name="moveit_panda")
        .robot_description_semantic("config/multi_arm.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .to_moveit_configs()
    )
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


   
    xacro_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots/multi_arm/multi_arm.urdf.xacro"
    )
    robot_description = xacro.process_file(
        xacro_path,
        mappings={"arm_id":"fr3", "hand":"true", "ee_id":"franka_hand"}
    ).toxml()
    moveit_config.robot_description = {"robot_description": robot_description}
    servo_yaml = load_yaml("moveit_panda", "config/left_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    

    return LaunchDescription([

       
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}, {"use_sim_time": True}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
          
        ),

      
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
                {"moveit_controller_manager":"moveit_simple_controller_manager/MoveItSimpleControllerManager"},
                {"moveit_simple_controller_manager.fake_controllers":
                   os.path.join(get_package_share_directory("moveit_panda"), "config", "ros2_controllers.yaml")},
                {"allow_trajectory_execution": True},
                {"fake_execution_type": "interpolate"},
                {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
                {"use_sim_time": True},
            ],
            output="screen",
        ),
        


      
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                os.path.join(get_package_share_directory("moveit_panda"), "config", "ros2_controllers.yaml"),
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"robot_description": robot_description},
            
            ],
            output="screen",
        ),

      
        TimerAction(
            period=3.0,  
            actions=[
                
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
            ]
        ),
    
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[
                {"robot_description": robot_description},
                moveit_config.robot_description_semantic,
                {"use_sim_time": True},
            ],
            arguments=["-d", os.path.join(get_package_share_directory("moveit_panda"), "config", "moveit.rviz")],
            output="screen",
        ),

        Node(package="robot_bringup", executable="ee_marker"),
        Node(package="robot_bringup", executable="hand_tracking"),
        Node(package="robot_bringup", executable="teleop_moveit"),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="moveit_servo",
                    executable="servo_node_main",
                    name="servo_left",
                    output="screen",
                    parameters=[
                        #os.path.join(get_package_share_directory("moveit_panda"), "config", "left_servo.yaml"),
                        servo_params,
                        {"robot_description": robot_description},
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        {"use_sim_time": True}
                    ],

                ),
            # """   Node(
            #        package="moveit_servo",
            #        executable="servo_node_main",
            #        name="servo_right",
            #        output="screen",
            #        parameters=[
            #            os.path.join(get_package_share_directory("moveit_panda"), "config", "right_servo.yaml"),
            #            {"robot_description": robot_description},
            #            moveit_config.robot_description_semantic,
            #            moveit_config.robot_description_kinematics,
            #            {"use_sim_time": True}
            #        ],
#
            #    ),"""
    ]
)
    ])
