import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():


    
    """
    DESCLAIMER : If the servo does not start, Please user below ros2 command to trigger it in a new terminal.
 
    Always use $ros2 service call /servo_left/start_servo std_srvs/srv/Trigger {} to start the servo motor.


    Nodes Runnnig (In order of launch) : 

    1) Robot State Publisher
    2) Joint State Broadcaster
    3) Mover_group (Responsible for Motion Planning)
    4) Controller Manager (We are loading Controllers manually also/ Sanity check)
    5) Rviz2
    6) EE_marker from robot bringup package
    7) Hand Tracking
    8) Hand Teleop (responsible for sending delta twist to move it servo)
    9) Left and Right Moveit Servo nodes.

    Note about Moveit servo: in the config file of the params, the servo params should be structured down with an 
    indentation of name called moveit_servo (If configured with Yaml, the launch file can be shortened).


    """

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
        except (
            EnvironmentError
        ):  
            return None

    def load_yaml(package_name, file_path):
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)

        try:
            with open(absolute_file_path, "r") as file:
                return yaml.safe_load(file)
        except (
            EnvironmentError
        ):  
            return None

    xacro_path = os.path.join(
        get_package_share_directory("franka_description"),
        "robots/multi_arm/multi_arm.urdf.xacro",
    )
    robot_description = xacro.process_file(
        xacro_path, mappings={"arm_id": "fr3", "hand": "true", "ee_id": "franka_hand"}
    ).toxml()
    moveit_config.robot_description = {"robot_description": robot_description}
    # servo_yaml = load_yaml("moveit_panda", "config/left_servo.yaml")
    # servo_params = {"moveit_servo": servo_yaml}
    servo_params = load_yaml("moveit_panda", "config/servo.yaml")

    kinematics_yaml = load_yaml("moveit_panda", "config/kinematics.yaml")
    kinematics_params = {"robot_description_kinematics": kinematics_yaml}

    start_right_servo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/servo_right/start_servo",
                    'std_srvs/srv/Trigger "{}"',
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": True},
                ],
            ),
            #Node(
            #    package="joint_state_publisher_gui",
            #    executable="joint_state_publisher_gui",
            #    parameters=[{"use_sim_time": True}],
            #),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
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
                    {
                        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
                    },
                    {
                        "moveit_simple_controller_manager.fake_controllers": os.path.join(
                            get_package_share_directory("moveit_panda"),
                            "config",
                            "ros2_controllers.yaml",
                        )
                    },
                    {"allow_trajectory_execution": True},
                    {"trajectory_execution.allowed_execution_duration_scaling": 1.2},
                    {"use_sim_time": True},
                ],
                output="screen",
                # arguments=['--ros-args', '--log-level', 'debug'],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    os.path.join(
                        get_package_share_directory("moveit_panda"),
                        "config",
                        "ros2_controllers.yaml",
                    ),
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {"robot_description": robot_description},
                ],
                output="screen",
                # arguments=['--ros-args', '--log-level', 'debug'],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda1_arm_controller",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda2_arm_controller",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda1_gripper_trajectory_controller",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda2_gripper_trajectory_controller",
                            "--controller-manager",
                            "/controller_manager",
                        ],
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                parameters=[
                    {"robot_description": robot_description},
                    moveit_config.robot_description_semantic,
                    {"use_sim_time": True},
                    load_yaml("moveit_panda", "config/kinematics.yaml"),
                ],
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("moveit_panda"),
                        "config",
                        "moveit.rviz",
                    ),
                ],
                output="screen",
            ),
            #Node(package="robot_bringup", executable="ee_marker"),
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
                            # os.path.join(get_package_share_directory("moveit_panda"), "config", "left_servo.yaml"),
                            servo_params,
                            {
                               
                                "moveit_servo.use_gazebo": False,
                                "moveit_servo.command_in_type": "speed_units",
                                "moveit_servo.scale.linear": 0.6,
                                "moveit_servo.scale.rotational": 0.3,
                                "moveit_servo.scale.joint": 0.005,
                                "moveit_servo.publish_period": 0.0175,
                                "moveit_servo.command_out_type": "trajectory_msgs/JointTrajectory",
                                "moveit_servo.publish_joint_positions": True,
                                "moveit_servo.publish_joint_velocities": False,
                                "moveit_servo.publish_joint_accelerations": False,
                                "moveit_servo.smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
                                "moveit_servo.smoothing_filter_params": {
                                    "order": 2},
                                "moveit_servo.move_group_name": "panda1_arm",
                                "moveit_servo.planning_frame": "base",
                                "moveit_servo.ee_frame_name": "left_fr3_link8",
                                "moveit_servo.robot_link_command_frame": "left_fr3_link8",
                                "moveit_servo.incoming_command_timeout": 0.1,
                                "moveit_servo.num_outgoing_halt_msgs_to_publish": 4,
                                "moveit_servo.lower_singularity_threshold": 17.0,
                                "moveit_servo.hard_stop_singularity_threshold": 30.0,
                                "moveit_servo.joint_limit_margin": 0.1,
                                "moveit_servo.leaving_singularity_threshold_multiplier": 2.0,
                                "moveit_servo.cartesian_command_in_topic": "~/delta_twist_cmds",
                                "moveit_servo.joint_command_in_topic": "~/delta_joint_cmds",
                                "moveit_servo.joint_topic": "/joint_states",
                                "moveit_servo.status_topic": "~/status",
                                "moveit_servo.command_out_topic": "/panda1_arm_controller/joint_trajectory",
                                "moveit_servo.check_collisions": True,
                                "moveit_servo.collision_check_rate": 10.0,
                                "moveit_servo.self_collision_proximity_threshold": 0.01,
                                "moveit_servo.scene_collision_proximity_threshold": 0.02,
                            },
                            {"use_intra_process_comms": True},
                            {"robot_description": robot_description},
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                        ],
                        remappings=[
                            (
                                "joint_command",
                                "/panda1_arm_controller/joint_trajectory",
                            ),
                            ("status", "/servo_left/status"),
                        ],
                      
                    ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="moveit_servo",
                        executable="servo_node_main",
                        name="servo_right",
                        output="screen",
                        parameters=[
                            # os.path.join(get_package_share_directory("moveit_panda"), "config", "left_servo.yaml"),
                            servo_params,
                            {
                                "moveit_servo.use_gazebo": False,
                                "moveit_servo.command_in_type": "speed_units",
                                "moveit_servo.scale.linear": 0.6,
                                "moveit_servo.scale.rotational": 0.3,
                                "moveit_servo.scale.joint": 0.01,
                                "moveit_servo.publish_period": 0.0175,
                                "moveit_servo.command_out_type": "trajectory_msgs/JointTrajectory",
                                "moveit_servo.publish_joint_positions": True,
                                "moveit_servo.publish_joint_velocities": False,
                                "moveit_servo.publish_joint_accelerations": False,
                                "moveit_servo.smoothing_filter_plugin_name": "online_signal_smoothing::ButterworthFilterPlugin",
                                "moveit_servo.move_group_name": "panda2_arm",
                                "moveit_servo.planning_frame": "base",
                                "moveit_servo.ee_frame_name": "right_fr3_link8",
                                "moveit_servo.robot_link_command_frame": "right_fr3_link8",
                                "moveit_servo.incoming_command_timeout": 0.1,
                                "moveit_servo.num_outgoing_halt_msgs_to_publish": 4,
                                "moveit_servo.lower_singularity_threshold": 17.0,
                                "moveit_servo.hard_stop_singularity_threshold": 30.0,
                                "moveit_servo.joint_limit_margin": 0.1,
                                "moveit_servo.leaving_singularity_threshold_multiplier": 2.0,
                                "moveit_servo.cartesian_command_in_topic": "~/delta_twist_cmds",
                                "moveit_servo.joint_command_in_topic": "~/delta_joint_cmds",
                                "moveit_servo.joint_topic": "/joint_states",
                                "moveit_servo.status_topic": "~/status",
                                "moveit_servo.command_out_topic": "/panda2_arm_controller/joint_trajectory",
                                "moveit_servo.check_collisions": True,
                                "moveit_servo.collision_check_rate": 10.0,
                                "moveit_servo.self_collision_proximity_threshold": 0.01,
                                "moveit_servo.scene_collision_proximity_threshold": 0.02,
                            },
                            {"use_intra_process_comms": True},
                            {"robot_description": robot_description},
                            moveit_config.robot_description_semantic,
                            load_yaml("moveit_panda", "config/kinematics.yaml"),
                        ],
                        remappings=[
                            (
                                "joint_command",
                                "/panda2_arm_controller/joint_trajectory",
                            ),
                            ("status", "/servo_right/status"),
                        ],
                      
                    ),
                 
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/servo_left/start_servo",
                            "std_srvs/srv/Trigger", 
                            "{}",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/servo_right/start_servo",
                            "std_srvs/srv/Trigger", 
                            "{}",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
])