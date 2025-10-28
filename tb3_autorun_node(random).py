from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

class SimpleMove(Node):
    def __init__(self):
        super().__init__('simple_move')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move)

    def move(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.2
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()