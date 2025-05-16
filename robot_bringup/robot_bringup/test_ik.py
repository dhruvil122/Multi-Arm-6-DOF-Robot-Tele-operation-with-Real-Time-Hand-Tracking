import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration

class ComputeIKTest(Node):
    def __init__(self):
        super().__init__('compute_ik_test')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')
     
        self.last_solution = None  
       
        self.test_pose()

    def test_pose(self):
       
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'panda1_arm'
        req.ik_request.ik_link_name = 'left_fr3_link6'
    
        pose = PoseStamped()
        pose.header.frame_id = 'left_fr3_link0'
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0
        req.ik_request.pose_stamped = pose


        joint_names = [
            'left_fr3_joint1', 'left_fr3_joint2', 'left_fr3_joint3',
            'left_fr3_joint4', 'left_fr3_joint5', 'left_fr3_joint6'
        ]
        req.ik_request.robot_state.joint_state.name = joint_names
        if self.last_solution:
            req.ik_request.robot_state.is_diff = False
            req.ik_request.robot_state.joint_state.position = self.last_solution
        else:
            req.ik_request.robot_state.is_diff = True
            req.ik_request.robot_state.joint_state.position = [0.0]*6

   
        req.ik_request.timeout = Duration(sec=0, nanosec=500_000_000)

   
        self.get_logger().info("Sending IK request...")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

   
        res = future.result()
        if not res:
            self.get_logger().error(" No response from IK service.")
            return

        self.get_logger().info(f"Response error_code: {res.error_code.val}")
        if res.solution.joint_state.name:
            self.get_logger().info(" IK solution found:")
            for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position):
                self.get_logger().info(f"  {name}: {pos:.3f}")
            # Cache for next time
            self.last_solution = list(res.solution.joint_state.position)
        else:
            self.get_logger().error(" No joint_state returned.")

def main(args=None):
    rclpy.init(args=args)
    node = ComputeIKTest()
    node.destroy_node()
    rclpy.shutdown()

