#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration


class TeleopWithMoveGroup(Node):
    def __init__(self):
       
        
        super().__init__('teleop_with_movegroup')

        
        """self._mg_cli = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Waiting for MoveGroup action server…")
        self._mg_cli.wait_for_server()
        self.get_logger().info("✔ MoveGroup action server available")
        """

        self.pub_left = self.create_publisher(PoseStamped,"servo_left/pose_cmds",10)
        self.pub_right = self.create_publisher(PoseStamped,"servo_right/pose_cmds",10)
        self.create_subscription(
            PoseStamped, '/left_teleop_target_pose',self.left_callback,10
        )
        self.create_subscription(
            PoseStamped, '/right_teleop_target_pose',self.right_callback,10
        )

    def left_callback (self,m:PoseStamped):
        m.header.frame_id = 'left_fr3_link0'
        self.pub_left.publish(m)
        self.get_logger().info("Publishing Pose Stamps for left robot")

    def right_callback (self,m:PoseStamped):
        m.header.frame_id = 'right_fr3_link0'
        self.pub_right.publish(m)





        """----------------Not Working - The movegroup is not in realtime hence not included in the final submission------------------"""

    """def _on_pose(self, pose: PoseStamped, group: str, ee_link: str):
   
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 0.5

      
        c = Constraints()
       
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = ee_link
        pc.weight = 1.0
       
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        tol = 0.01
        box.dimensions = [tol*2, tol*2, tol*2]
        pc.constraint_region.primitives.append(box)
        region_pose = Pose()
        region_pose.position = pose.pose.position
        region_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(region_pose)
     
        c.position_constraints.append(pc)

    
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = ee_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = tol
        oc.absolute_y_axis_tolerance = tol
        oc.absolute_z_axis_tolerance = tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(c)

        self.get_logger().info(f"sending MoveGroup goal for {group}")
        send_goal = self._mg_cli.send_goal_async(goal)
        send_goal.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MoveGroup goal was rejected")
            return
        self.get_logger().info("MoveGroup goal accepted, awaiting result…")
        handle.get_result_async().add_done_callback(self._on_move_result)

    def _on_move_result(self, future):
        result = future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("plan+execute succeeded")
        else:
            self.get_logger().error(f"plan+execute failed: {result.error_code.val}")
"""

def main():
    rclpy.init()
    node = TeleopWithMoveGroup()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
