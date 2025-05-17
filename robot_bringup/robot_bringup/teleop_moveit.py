#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TwistStamped

class TeleopWithMoveGroup(Node):
    def __init__(self):
        super().__init__('teleop_with_movegroup')

        self.pub_left = self.create_publisher(TwistStamped, "servo_left/delta_twist_cmds", 10)
        self.sub_left = self.create_subscription(
            PoseStamped, "/left_teleop_target_pose", self.left_callback, 10
        )

        self.prev_time = None
        self.prev_pos = None
        self.prev_euler = None
        self.joint_traj_sub = self.create_subscription(
            JointTrajectory,
            '/servo_left/joint_command',
            self.relay_trajectory,
            10
        )
        self.client = ActionClient(self, FollowJointTrajectory, '/panda1_arm_controller/follow_joint_trajectory')


    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return np.array([X, Y, Z])  # Radians

    def left_callback(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        ori = msg.pose.orientation
        euler = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_pos = pos
            self.prev_euler = euler
            return

        dt = current_time - self.prev_time
        if dt <= 0:
            return

        lin_vel = (pos - self.prev_pos) / dt
        ang_vel = (euler - self.prev_euler) / dt

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "left_fr3_link0"
        twist.twist.linear.x = float(lin_vel[0])
        twist.twist.linear.y = float(lin_vel[1])
        twist.twist.linear.z = float(lin_vel[2])
        twist.twist.angular.x = float(ang_vel[0])
        twist.twist.angular.y = float(ang_vel[1])
        twist.twist.angular.z = float(ang_vel[2])
        MAX_LINEAR_VEL = 0.1  # m/s
        MAX_ANGULAR_VEL = 0.05  # rad/s

        twist.twist.linear.x = max(min(twist.twist.linear.x, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
        twist.twist.linear.y = max(min(twist.twist.linear.y, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
        twist.twist.angular.z = max(min(twist.twist.angular.z, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

        self.pub_left.publish(twist)
        print(twist)

        self.prev_time = current_time
        self.prev_pos = pos
        self.prev_euler = euler


    def relay_trajectory(self, msg):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg
        self.client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = TeleopWithMoveGroup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
