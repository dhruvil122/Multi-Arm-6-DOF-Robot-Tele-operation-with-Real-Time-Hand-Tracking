import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

class CircleMarkerPublisher(Node):
    def __init__(self):
        super().__init__('circle_marker_publisher')
        self.publisher = self.create_publisher(Marker, '/ee_circle_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
      

        left_marker = Marker()
        left_marker.header.frame_id = "left_fr3_hand"
        left_marker.header.stamp = node.get_clock().now().to_msg()
        left_marker.ns = "left_ee_marker"
        left_marker.id = 0
        left_marker.type = Marker.SPHERE
        left_marker.action = Marker.ADD

     
        left_marker.pose.position.x = 0.0
        left_marker.pose.position.y = 0.0
        left_marker.pose.position.z = 0.0

       
        left_marker.pose.orientation.x = 0.0
        left_marker.pose.orientation.y = 0.0
        left_marker.pose.orientation.z = 0.0
        left_marker.pose.orientation.w = 1.0

      
        left_marker.scale.x = 0.15 
        left_marker.scale.y = 0.15
        left_marker.scale.z = 0.15

        left_marker.color.a = 0.8  
        left_marker.color.r = 0.1
        left_marker.color.g = 0.6
        left_marker.color.b = 1.0


        radius = 0.05
        steps = 100
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            left_marker.points.append(
                Point(x=x, y=y, z=0.0)
            )

        self.publisher.publish(left_marker)
        
        
        right_marker = Marker()
        right_marker.header.frame_id = "right_fr3_hand"
        right_marker.header.stamp = node.get_clock().now().to_msg()
        right_marker.ns = "right_ee_marker"
        right_marker.id = 0
        right_marker.type = Marker.SPHERE
        right_marker.action = Marker.ADD

   
        right_marker.pose.position.x = 0.0
        right_marker.pose.position.y = 0.0
        right_marker.pose.position.z = 0.0

   
        right_marker.pose.orientation.x = 0.0
        right_marker.pose.orientation.y = 0.0
        right_marker.pose.orientation.z = 0.0
        right_marker.pose.orientation.w = 1.0

   
        right_marker.scale.x = 0.15  
        right_marker.scale.y = 0.15
        right_marker.scale.z = 0.15

        right_marker.color.a = 0.8  
        right_marker.color.r = 0.1
        right_marker.color.g = 0.6
        right_marker.color.b = 1.0


        radius = 0.05
        steps = 100
        for i in range(steps + 1):
            angle = 2 * math.pi * i / steps
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            right_marker.points.append(
                Point(x=x, y=y, z=0.0)
            )

        self.publisher.publish(right_marker)

rclpy.init()
node = CircleMarkerPublisher()
rclpy.spin(node)
