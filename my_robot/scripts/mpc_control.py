import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
import cvxpy as cp
import time


class MPCNavigatorNode(Node):
    def __init__(self):
        super().__init__("mpc_navigator")

        self.bridge = CvBridge()

        self.subscription_image = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.subscription_depth = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.depth_data = None

        self.marker_center = None

        self.target_distance = 0.5  # Target distance from the marker (in meters)

        # Movement timing

        self.start_time = None

        self.elapsed_time = None

        self.is_moving = False

        # MPC parameters

        self.T = 0.1  # Sampling time

        self.N = 20  # Prediction horizon

        self.Q = np.diag([2.0, 2.0, 5.0])  # State cost matrix

        self.R = np.diag([0.01, 0.7])  # Control cost matrix

        # Constraints

        self.v_max = 0.5

        self.omega_max = 0.5

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.aruco_params = cv2.aruco.DetectorParameters()

    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            self.depth_data = np.nan_to_num(depth_image, nan=np.inf)

            depth_display = cv2.normalize(
                self.depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )

            depth_colormap = cv2.applyColorMap(depth_display, cv2.COLORMAP_BONE)

            cv2.namedWindow("Depth Camera", cv2.WINDOW_NORMAL)

            cv2.resizeWindow(
                "Depth Camera", 640, 480
            )  # Adjust width and height as needed

            # Show timer information

            if self.start_time is not None and self.is_moving:
                current_time = time.time() - self.start_time

                cv2.putText(
                    depth_colormap,
                    f"Time: {current_time:.2f} s",
                    (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    3,
                    (255, 0, 0),
                    3,
                )

            elif not self.is_moving and self.elapsed_time is not None:
                cv2.putText(
                    depth_colormap,
                    f"Final Time: {self.elapsed_time:.2f} s",
                    (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    3,
                    (0, 255, 0),
                    3,
                )

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
                    5,
                    (0, 255, 0),
                    5,
                )

            cv2.imshow("Depth Camera", depth_colormap)

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

            if ids is not None:
                # Draw detected markers on the frame

                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                self.marker_center = np.mean(corners[0][0], axis=0)

            else:
                self.marker_center = None

            # Create a named window of size 640x480

            cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

            cv2.resizeWindow("Camera Feed", 640, 480)

            # Display the frame in the named window

            cv2.imshow("Camera Feed", frame)

            cv2.waitKey(1)  # Wait for 1 millisecond

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def kinematic_model(self, x, u):
        e, phi, alpha = x

        v, omega = u

        return cp.vstack(
            [
                e - v * cp.multiply(cp.cos(alpha), self.T),
                phi + cp.multiply(v, cp.sin(alpha)) * self.T / e,
                alpha + cp.multiply(v, cp.sin(alpha)) * self.T / e - omega * self.T,
            ]
        )

    def linearize_model(self, x, u):
        e, phi, alpha = x

        v, omega = u

        A = np.array(
            [
                [1, 0, v * self.T * np.sin(alpha)],
                [0, 1, -v * self.T * np.cos(alpha) / e],
                [0, 0, 1 - v * self.T * np.cos(alpha) / e],
            ]
        )

        B = np.array(
            [
                [-self.T * np.cos(alpha), 0],
                [self.T * np.sin(alpha) / e, 0],
                [self.T * np.sin(alpha) / e, -self.T],
            ]
        )

        c = np.array(
            [
                -v * self.T * np.cos(alpha),
                v * self.T * np.sin(alpha) / e,
                v * self.T * np.sin(alpha) / e - omega * self.T,
            ]
        )

        return A, B, c

    def mpc_problem(self, x0, x_ref):
        N = self.N

        x = cp.Variable((N + 1, 3))

        u = cp.Variable((N, 2))

        cost = 0

        constraints = [x[0] == x0]

        for t in range(N):
            A, B, c = self.linearize_model(
                x0, [0, 0]
            )  # Linearize around current state and zero input

            cost += cp.quad_form(x[t] - x_ref, self.Q) + cp.quad_form(u[t], self.R)

            constraints += [
                x[t + 1] == A @ x[t] + B @ u[t] + c,
                cp.abs(u[t, 0]) <= self.v_max,
                cp.abs(u[t, 1]) <= self.omega_max,
            ]

        cost += cp.quad_form(x[N] - x_ref, self.Q)

        problem = cp.Problem(cp.Minimize(cost), constraints)

        problem.solve(solver=cp.OSQP, verbose=False)

        if problem.status != cp.OPTIMAL:
            self.get_logger().warn(f"MPC solver status: {problem.status}")

            return np.zeros(2)

        return u[0].value

    def timer_callback(self):
        twist = Twist()

        if self.marker_center is not None and self.depth_data is not None:
            height, width = self.depth_data.shape

            cx, cy = int(self.marker_center[0]), int(self.marker_center[1])

            cx = np.clip(cx, 0, width - 1)

            cy = np.clip(cy, 0, height - 1)

            marker_depth = self.depth_data[cy, cx]

            # Stop if the robot is within 0.54 meters of the marker

            if marker_depth <= 1.4:
                twist.linear.x = 0.0

                twist.angular.z = 0.0

                self.get_logger().info("Target reached. Stopping.")

                # Record elapsed time when stopping

                if self.is_moving:
                    self.is_moving = False

                    self.elapsed_time = time.time() - self.start_time

                    self.start_time = None

            else:
                # Start timer if robot starts moving

                if not self.is_moving:
                    self.is_moving = True

                    self.start_time = time.time()

                e = marker_depth

                phi = np.arctan2(-cy + height / 2, -cx + width / 2)

                alpha = phi  # Assuming robot's orientation is aligned with camera

                x0 = np.array([e, phi, alpha])

                x_ref = np.array([self.target_distance, 0, 0])

                u = self.mpc_problem(x0, x_ref)

                twist.linear.x = u[0]

                twist.angular.z = u[1]

                self.get_logger().info(
                    f"MPC control: v={twist.linear.x:.3f}, ω={twist.angular.z:.3f}"
                )

        else:
            twist.linear.x = 0.0

            twist.angular.z = 0.0

            self.get_logger().info("No marker detected or no depth data.")

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MPCNavigatorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
