import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class CmdVel(Node):

    def __init__(self):
        super().__init__('cmd_pub')

        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_vel)

    def publish_vel(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Twist
        msg.twist.linear.x = 0.5
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
