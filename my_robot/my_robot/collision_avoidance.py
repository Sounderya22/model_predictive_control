import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
import time  # Import for timing


class ArucoNavigatorNode(Node):
    def __init__(self):
        super().__init__("aruco_navigator")

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        # Subscriptions
        self.subscription_image = self.create_subscription(
            Image, "/camera1/image_raw", self.image_callback, 10
        )
        self.subscription_depth = self.create_subscription(
            Image, "/camera1/depth/image_raw", self.depth_callback, 10
        )

        # Publisher
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Internal state
        self.depth_data = None
        self.marker_center = None
        self.target_distance = 0.5  # Target distance from the marker (in meters)
        self.start_time = None  # Initialize timer start
        self.reached_target = False  # Flag to track if target is reached
        self.elapsed_time = None  # Store elapsed time when target is reached

        # Prompt the user for PD controller parameters
        self.Kp_linear = 0.028
        self.Kd_linear = 0.000001
        self.Kp_angular = 0.001
        self.Kd_angular = 0.001

        # Previous errors for derivative control
        self.prev_depth_error = 0.0
        self.prev_angular_error = 0.0

        # Load ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

    def depth_callback(self, msg):
        """Process depth image and update depth data."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_data = np.nan_to_num(
                depth_image, nan=np.inf
            )  # Replace NaNs with infinity

            # Visualize the depth image
            depth_display = cv2.normalize(
                self.depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_BONE)

            # Overlay distance if marker center is available
            if self.marker_center is not None:
                cx, cy = int(self.marker_center[0]), int(self.marker_center[1])
                cx = np.clip(cx, 0, self.depth_data.shape[1] - 1)
                cy = np.clip(cy, 0, self.depth_data.shape[0] - 1)

                marker_depth = self.depth_data[cy, cx]
                cv2.putText(
                    depth_colormap,
                    f"Depth: {marker_depth:.2f}m",
                    (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

            # Overlay the running timer on the depth image
            if self.start_time is not None and not self.reached_target:
                current_time = time.time() - self.start_time
                cv2.putText(
                    depth_colormap,
                    f"Time: {current_time:.2f} s",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 0),
                    2,
                )
            elif self.reached_target and self.elapsed_time is not None:
                cv2.putText(
                    depth_colormap,
                    f"Final Time to reach 0.54m (Limit of PD sensor ): {self.elapsed_time:.2f} s",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )

            cv2.imshow("Depth Camera Feed", depth_colormap)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        """Process RGB image and detect ArUco marker."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                marker_corners = corners[0]
                self.marker_center = np.mean(marker_corners[0], axis=0)
                cx, cy = int(self.marker_center[0]), int(self.marker_center[1])
                self.get_logger().info(
                    f"Marker detected at {self.marker_center} with ID: {ids[0]}"
                )
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Overlay the marker coordinates
                cv2.putText(
                    frame,
                    f"Coords: ({cx}, {cy})",
                    (cx, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )
            else:
                self.marker_center = None
                self.get_logger().info("No marker detected.")

            cv2.imshow("Camera Feed with Markers", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def timer_callback(self):
        """PD Control loop to navigate to the ArUco marker."""
        twist = Twist()

        if self.marker_center is not None and self.depth_data is not None:
            height, width = self.depth_data.shape
            cx, cy = int(self.marker_center[0]), int(self.marker_center[1])

            # Ensure coordinates are within image bounds
            cx = np.clip(cx, 0, width - 1)
            cy = np.clip(cy, 0, height - 1)

            # Check the depth of the marker
            marker_depth = self.depth_data[cy, cx]
            self.get_logger().info(f"Marker depth: {marker_depth:.2f}m")

            # Start the timer when the robot first moves
            if self.start_time is None and marker_depth > 0.55:
                self.start_time = (
                    time.time()
                )  # Start timing when the robot starts moving
                self.get_logger().info("Timer started.")

            # Calculate errors
            depth_error = marker_depth - self.target_distance
            angular_error = cx - width // 2  # Horizontal offset from image center

            # Calculate derivatives
            depth_derivative = depth_error - self.prev_depth_error
            angular_derivative = angular_error - self.prev_angular_error

            # PD control for linear velocity
            twist.linear.x = (
                self.Kp_linear * depth_error + self.Kd_linear * depth_derivative
            )

            # PD control for angular velocity
            twist.angular.z = -(
                self.Kp_angular * angular_error + self.Kd_angular * angular_derivative
            )

            # Update previous errors
            self.prev_depth_error = depth_error
            self.prev_angular_error = angular_error

            self.get_logger().info("Navigating to marker using PD control.")

            # Stop the timer when the robot reaches close to 0.55m
            if marker_depth <= 0.54 and not self.reached_target:
                self.reached_target = True
                end_time = time.time()
                self.elapsed_time = end_time - self.start_time
                self.get_logger().info(
                    f"Reached target. Time taken: {self.elapsed_time:.2f} seconds."
                )
        else:
            # Stop if no marker is detected
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("No marker detected or no depth data.")

        # Publish the Twist message
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
