import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class RotateRobotNode(Node):
    def __init__(self):
        super().__init__('rotate_robot_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rotate_end_pub = self.create_publisher(Bool, '/rotate_end', 10)  # 퍼블리셔 추가

        self.rotation_speed = 0.2  # rad/s
        self.rotation_duration = 2 * np.pi / self.rotation_speed
        self.rotation_start_time = self.get_clock().now().nanoseconds / 1e9

        self.rotation_done = False

        self.get_logger().info(f'Starting 1 full rotation for {self.rotation_duration:.1f} seconds.')

        self.timer = self.create_timer(0.1, self.rotate)

    def rotate(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.rotation_start_time

        twist = Twist()
        if elapsed < self.rotation_duration:
            twist.angular.z = self.rotation_speed
            self.cmd_vel_pub.publish(twist)
        else:
            if not self.rotation_done:
                # 회전 끝났다고 신호 보내기
                self.get_logger().info('Rotation complete. Publishing rotate_end signal.')
                self.rotate_end_pub.publish(Bool(data=True))
                self.rotation_done = True

            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            # 여기서 바로 종료하면 안됨. 종료는 main spin 이후에.
            # rclpy.shutdown() 호출하지 말고, 종료는 외부에서 해도 됨.

def main(args=None):
    rclpy.init(args=args)
    node = RotateRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
