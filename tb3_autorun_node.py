import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class SimpleAutoNav(Node):
    def __init__(self):
        super().__init__('simple_auto_nav')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.obstacle_ahead = False

    def scan_callback(self, msg):
        front = min(msg.ranges[0:20] + msg.ranges[-20:])  # 前方約40°の距離を取得
        self.obstacle_ahead = (front < 0.4)  # 40cm以内に障害物があると判断

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.obstacle_ahead:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.8  # 旋回して回避
        else:
            msg.twist.linear.x = 0.1
            msg.twist.angular.z = 0.0  # まっすぐ進む

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAutoNav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()