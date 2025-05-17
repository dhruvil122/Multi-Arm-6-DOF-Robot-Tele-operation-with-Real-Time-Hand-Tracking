import cv2
import rclpy
import math
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose2D
import numpy as np
import mediapipe as mp 
import mediapipe.python.solutions.drawing_styles as drawing_styles

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        self.publisher_left = self.create_publisher(Pose2D, '/hand_tracking/panda1/pose2d', 10)
        self.publisher_right = self.create_publisher(Pose2D, '/hand_tracking/panda2/pose2d', 10)
        self.left_pose_pub = self.create_publisher(PoseStamped, '/left_teleop_target_pose', 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, '/right_teleop_target_pose', 10)

        
        self.cap = cv2.VideoCapture("http://192.168.64.1:5000/video_feed")

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2,min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

        self.timer = self.create_timer(1.0 / 60.0, self.process_frame)
        self.prev_left = np.array([0.0,0.0])
        self.prev_right = np.array([0.0,0.0])
        self.alpha = 0.7

    def lowpass(self,current:np.ndarray,previous:np.ndarray) -> np.ndarray:
        return self.alpha * current + (1-self.alpha) * previous

        
    def euler_to_quaternion(self,yaw, pitch, roll):

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            return [qx, qy, qz, qw]
   

    def process_frame(self):

       
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame from camera.")
        if success:
            frame = cv2.resize(frame, (1280, 720))
            frame = cv2.flip(frame, 1)

            
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)
        frame_height, frame_width, _ = frame.shape 
        #print(f'frame shape is {frame.shape}')

        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label  
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                index_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS,drawing_styles.get_default_hand_landmarks_style(),
                drawing_styles.get_default_hand_connections_style(),)

                msg = Pose2D()
                #msg.x = (index_finger_tip.x - 0.5) 
                #msg.y = (-index_finger_tip.y + 0.5) 
               
                
                #print(msg.x,msg.y)
                raw = np.array([index_finger_tip.x,index_finger_tip.y])

                #print(f'raw coordinates in pose2d is x = {msg.x} and y = {msg.y}')

                if label.lower() == 'left':
                    """msg.x = msg.x - 0.90
                    if msg.x < 0:
                        msg.x = -msg.x   ///Not needed if using two robots. Tested, does not make sense
                    else:
                        pass"""
                    self.publisher_left.publish(msg)
                    
                    filtered_coordinates = self.lowpass(raw,self.prev_left) 
                    self.prev_left = filtered_coordinates
                    #self.get_logger().info(f"Published left hand: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
                    pose_l = PoseStamped()
                    pose_l.header.stamp = rclpy.time.Time().to_msg()
                    x,y = filtered_coordinates
                    self.get_logger().info(f"Published left hand: x={x:.2f}, y={y:.2f}")
                    theta = math.atan2(y,x)
                    pose_l.header.frame_id = 'left_fr3_link0'
                    pose_l.pose.position.x = x *2
                    pose_l.pose.position.y = y *2
                    pose_l.pose.position.z = 0.4 

                    q = self.euler_to_quaternion(theta, 0, 0)
                    pose_l.pose.orientation.x = q[0]
                    pose_l.pose.orientation.y = q[1]
                    pose_l.pose.orientation.z = q[2]
                    pose_l.pose.orientation.w = q[3]

                    self.left_pose_pub.publish(pose_l)
                  
                    print(pose_l)
                
                else:
                    self.publisher_right.publish(msg)
                    #self.get_logger().info(f"Published right hand: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
                    filtered_coordinates = self.lowpass(raw,self.prev_right)
                    self.prev_right = filtered_coordinates
                    x,y = filtered_coordinates
                    theta = math.atan2(y,x)
                    pose_r = PoseStamped()
                    pose_r.header.stamp = rclpy.time.Time().to_msg()

                    pose_r.header.frame_id = 'right_fr3_link0'
                    pose_r.pose.position.x = x *2
                    pose_r.pose.position.y = y *2
                    pose_r.pose.position.z = 0.4 
                    q = self.euler_to_quaternion(theta, 0, 0)
                    pose_r.pose.orientation.x = q[0]
                    pose_r.pose.orientation.y = q[1]
                    pose_r.pose.orientation.z = q[2]
                    pose_r.pose.orientation.w = q[3]

                    self.right_pose_pub.publish(pose_r)

        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    rclpy.spin(node)
   
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
