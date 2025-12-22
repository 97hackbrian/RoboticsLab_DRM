import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   w
a  s  d
   x

w/x : increase/decrease linear velocity (forward/backward)
a/d : increase/decrease angular velocity (left/right)
s   : force stop
CTRL-C to quit
"""

# Mapeo de teclas a movimientos (linear, angular)
moveBindings = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0),
}

SPEED_LINEAR = 0.5
SPEED_ANGULAR = 1.0  # rad/s


def getKey(settings):
    """Función para leer una tecla del buffer sin bloquear ni requerir Enter"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info("Teleop Node Started")

    def publish_velocity(self, target_linear_vel, target_angular_vel):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Twist
        msg.twist.linear.x = float(target_linear_vel)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(target_angular_vel)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    settings = termios.tcgetattr(sys.stdin)

    node = TeleopNode()

    target_linear_vel = 0.0
    target_angular_vel = 0.0

    print(msg)

    try:
        while rclpy.ok():
            key = getKey(settings)

            if key in moveBindings.keys():
                if key == 's':
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0
                else:
                    target_linear_vel = moveBindings[key][0] * SPEED_LINEAR
                    target_angular_vel = moveBindings[key][1] * SPEED_ANGULAR

                node.publish_velocity(target_linear_vel, target_angular_vel)
                print(
                    f"\rVel: Linear={target_linear_vel:.2f}, Angular={target_angular_vel:.2f}  ",
                    end="")

            elif key == '\x03':
                break

    except Exception as e:
        print(e)

    finally:
        node.publish_velocity(0.0, 0.0)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
