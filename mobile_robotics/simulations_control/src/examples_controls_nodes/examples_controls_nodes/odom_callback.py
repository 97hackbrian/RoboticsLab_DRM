import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCallback(Node):

    def __init__(self):
        super().__init__('odometry_sub')

        self.sub = self.create_subscription(Odometry, '/odometry',
                                            self.odom_callback, 10)

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        w = msg.twist.twist.angular.z

        self.get_logger().info(
            f"Position: x: {x:.2f} y: {y:.2f}  |  Vel: x:{vx:.2f} vy:{vy:.2f} z:{w:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = OdomCallback()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
