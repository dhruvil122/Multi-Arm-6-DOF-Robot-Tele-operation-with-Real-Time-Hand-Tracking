import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TeleopCartIK(Node):
    def __init__(self):
        super().__init__('teleop_cart_ik')

        # Subscribers for hand-tracked PoseStamped
        self.create_subscription(PoseStamped, '/left_teleop_target_pose', self.left_teleop_target_callback, 10)
        self.create_subscription(PoseStamped, '/right_teleop_target_pose', self.right_teleop_target_callback, 10)

        # IK Service Client
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

        # Publishers for joint trajectory to each arm controller
        self.left_joint_pub = self.create_publisher(JointTrajectory, '/panda1_arm_controller/joint_trajectory', 10)
        self.right_joint_pub = self.create_publisher(JointTrajectory, '/panda2_arm_controller/joint_trajectory', 10)

    def call_ik_service(self, group_name, joint_names, pose_msg):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.pose_stamped = pose_msg
        req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)
        req.ik_request.robot_state.joint_state.name = joint_names
        req.ik_request.robot_state.joint_state.position = [0.0] * len(joint_names)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().solution.joint_state.name:
            joint_state = future.result().solution.joint_state
            self.get_logger().info(f"IK succeeded for {group_name}")
            self.get_logger().info(f"Joint Names: {joint_state.name}")
            self.get_logger().info(f"Positions: {joint_state.position}")
            return joint_state
        else:
            self.get_logger().warn(f"IK failed for {group_name}")
            return None

    def publish_trajectory(self, joint_state, publisher):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = joint_state.name

        point = JointTrajectoryPoint()
        point.positions = joint_state.position
        point.time_from_start.sec = 1

        traj.points.append(point)
        publisher.publish(traj)
        self.get_logger().info("Published trajectory to controller.")

    def left_teleop_target_callback(self, msg):
        joint_names = [
            'left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3',
            'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6'
        ]
        joint_state = self.call_ik_service('panda1_arm', joint_names, msg)
        if joint_state:
            self.publish_trajectory(joint_state, self.left_joint_pub)

    def right_teleop_target_callback(self, msg):
        joint_names = [
            'right_fr3_joint1', 'right_fr3_joint2', 'right_fr3_joint3',
            'right_fr3_joint4', 'right_fr3_joint5', 'right_fr3_joint6'
        ]
        joint_state = self.call_ik_service('panda2_arm', joint_names, msg)
        if joint_state:
            self.publish_trajectory(joint_state, self.right_joint_pub)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopCartIK()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
