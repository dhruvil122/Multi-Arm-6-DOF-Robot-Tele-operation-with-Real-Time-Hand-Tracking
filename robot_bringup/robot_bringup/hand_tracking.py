import cv2
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Pose2D
import mediapipe as mp 

class HandTrackingNode(Node):
    def __init__(self):
        super().__init__('hand_tracking_node')
        self.publisher_left = self.create_publisher(Pose2D, '/hand_tracking/panda1/pose2d', 10)
        self.publisher_right = self.create_publisher(Pose2D, '/hand_tracking/panda2/pose2d', 10)

        
        self.cap = cv2.VideoCapture("http://192.168.64.1:5000/video_feed")

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2)
        self.mp_draw = mp.solutions.drawing_utils

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        frame = cv2.flip(frame, 1)
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                label = handedness.classification[0].label  
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                msg = Pose2D()
                msg.x = wrist.x
                msg.y = wrist.y
                msg.theta = 0.0

                if label.lower() == 'left':
                    self.publisher_left.publish(msg)
                    self.get_logger().info(f"Published left hand: x={msg.x:.2f}, y={msg.y:.2f}")
                else:
                    self.publisher_right.publish(msg)
                    self.get_logger().info(f"Published right hand: x={msg.x:.2f}, y={msg.y:.2f}")

        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HandTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
