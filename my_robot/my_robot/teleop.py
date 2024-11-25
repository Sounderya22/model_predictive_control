import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopDifferentialDrive(Node):

    def __init__(self):
        super().__init__('teleop_differential_drive')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info("Use WASD keys for movement: W=Forward, S=Backward, A=Turn Left, D=Turn Right")
        self.get_logger().info("Press SPACEBAR to stop the robot")
        self.get_logger().info("Press 'x' to quit")

        # Initialize velocities
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Velocity increment and limits
        self.linear_increment = 0.01
        self.angular_increment = 0.01
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def move_robot(self):
        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            if key == 'w':  # Increase linear forward speed
                self.linear_speed = min(self.linear_speed + self.linear_increment, self.max_linear_speed)
            elif key == 's':  # Increase linear backward speed
                self.linear_speed = max(self.linear_speed - self.linear_increment, -self.max_linear_speed)
            elif key == 'a':  # Increase angular left speed
                self.angular_speed = min(self.angular_speed + self.angular_increment, self.max_angular_speed)
            elif key == 'd':  # Increase angular right speed
                self.angular_speed = max(self.angular_speed - self.angular_increment, -self.max_angular_speed)
            elif key == ' ':  # Stop the robot
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            elif key == 'x':  # Exit the loop
                self.linear_speed = 0.0
                self.angular_speed = 0.0
                break
            else:
                continue

            # Publish the velocities
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.publisher_.publish(twist)

            self.get_logger().info(f"Linear Speed: {self.linear_speed:.2f}, Angular Speed: {self.angular_speed:.2f}")
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)

    teleop = TeleopDifferentialDrive()
    teleop.move_robot()

    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
