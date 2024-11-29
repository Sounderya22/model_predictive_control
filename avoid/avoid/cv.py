import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.depth_data = None

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_data = np.nan_to_num(depth_image, nan=10.0)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def timer_callback(self):
        if self.depth_data is not None:
            # Analyze depth data
            height, width = self.depth_data.shape
            roi = self.depth_data[height//2:, :]
            left = np.min(roi[:, :width//3])
            center = np.min(roi[:, width//3:2*width//3])
            right = np.min(roi[:, 2*width//3:])
            
            # Define avoidance logic
            twist = Twist()
            if center < 0.5:
                if left > right:
                    twist.angular.z = 0.5
                else:
                    twist.angular.z = -0.5
            else:
                twist.linear.x = 0.2
            
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
