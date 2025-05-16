import rclpy
from rclpy.node import Node
from moveit_py.core import RobotModelLoader

class IKChainValidator(Node):
    def __init__(self):
        super().__init__('ik_chain_validator')

        loader = RobotModelLoader(self, "robot_description")
        model = loader.get_model()
        if model is None:
            self.get_logger().error(" Failed to load robot model from robot_description.")
            return

        group = model.get_joint_model_group("panda1_arm")
        if group is None:
            self.get_logger().error(" Group 'panda1_arm' not found in SRDF.")
            return

        self.get_logger().info(f" Found group 'panda1_arm'")
        self.get_logger().info(f"Base link: {group.get_common_root()}")  # should be left_fr3_link0
        self.get_logger().info(f"End effector link: {group.get_link_model_names()[-1]}")  # should be left_fr3_link8

        joint_names = group.get_active_joint_model_names()
        self.get_logger().info(f"Active joints in chain: {joint_names}")

        if len(joint_names) != 6:
            self.get_logger().warn(" Expected 6 joints in the kinematic chain for a 6-DOF IK arm.")

def main(args=None):
    rclpy.init(args=args)
    node = IKChainValidator()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
