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
         

        """
        Topics :

        pub_left and pub_right : Publishes TwistStamps to delta_twist_cmds at respective node locations
        sub_left and sub_right : subscribes to the x,y and roll coming from the user hand
        traj_subscriber left and right : subscribes to the joint commands

        
        """
        self.pub_left = self.create_publisher(TwistStamped, "servo_left/delta_twist_cmds", 10)
        self.sub_left = self.create_subscription(
            PoseStamped, "/left_teleop_target_pose", self.left_callback, 10
        )
        self.left_joint_traj_sub = self.create_subscription(
            JointTrajectory,
            '/servo_left/joint_command',
            self.left_relay_trajectory,
            10
        )
        self.left_client = ActionClient(self, FollowJointTrajectory, '/panda1_arm_controller/follow_joint_trajectory')
        self.left_prev_time = None
        self.left_prev_pos = None
        self.left_prev_euler = None
        


        self.pub_right = self.create_publisher(TwistStamped,"servo_right/delta_twist_cmds",10)
        self.sub_right = self.create_subscription(PoseStamped,"/right_teleop_target_pose",self.right_callback,10)
        self.right_joint_traj_sub = self.create_subscription(
            JointTrajectory,
            "/servo_right/joint_command",
            self.right_relay_trajectory,
            10
        )
        self.right_client = ActionClient(self,FollowJointTrajectory,  'panda2_arm_controller/follow_joint_trajectory')
        self.right_prev_time = None
        self.right_prev_pos = None
        self.right_prev_euler = None


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
    
    def left_relay_trajectory(self, msg):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg
        self.left_client.send_goal_async(goal_msg)

    def left_callback(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        ori = msg.pose.orientation
        euler = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.left_prev_time is None:
            self.left_prev_time = current_time
            self.left_prev_pos = pos
            self.left_prev_euler = euler
            return

        dt = current_time - self.left_prev_time
        if dt <= 0:
            return

        lin_vel = (pos - self.left_prev_pos) / dt
        ang_vel = (euler - self.left_prev_euler) / dt

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

        self.left_prev_time = current_time
        self.left_prev_pos = pos
        self.left_prev_euler = euler

    def right_relay_trajectory(self, msg):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Controller action server not available.')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = msg
        self.right_client.send_goal_async(goal_msg)

    def right_callback(self, msg):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        ori = msg.pose.orientation
        euler = self.quaternion_to_euler(ori.x, ori.y, ori.z, ori.w)

        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.right_prev_time is None:
            self.right_prev_time = current_time
            self.right_prev_pos = pos
            self.right_prev_euler = euler
            return

        dt = current_time - self.right_prev_time
        if dt <= 0:
            return

        lin_vel = (pos - self.right_prev_pos) / dt
        ang_vel = (euler - self.right_prev_euler) / dt

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "right_fr3_link0"
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

        self.pub_right.publish(twist)
        print(twist)

        self.right_prev_time = current_time
        self.right_prev_pos = pos
        self.right_prev_euler = euler

    
def main():
    rclpy.init()
    node = TeleopWithMoveGroup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
