#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math

class ServoDirectionController(Node):
    def __init__(self):
        super().__init__('servo_direction_controller')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(JointState, '/servo_commands', 10)
        self.wheelbase = 0.6  # distancia entre ejes longitud
        self.trackwidth = 0.6 # distancia entre ruedas ancho

        # nombres de tus joints de los servos
        self.joint_names = ['joint_st1l','joint_st2l','joint_st1r','joint_st2r']

    def cmd_vel_callback(self, msg: Twist):
        L = self.wheelbase
        W = self.trackwidth
        r = math.hypot(L, W)

        # Cinemática simple: calcular ángulo de cada rueda
        A = msg.linear.x - msg.angular.z * (L/r)
        B = msg.linear.x + msg.angular.z * (L/r)
        C = msg.linear.y - msg.angular.z * (W/r)
        D = msg.linear.y + msg.angular.z * (W/r)

        angles = [
            math.atan2(D, B),  # joint_st1l
            math.atan2(C, B),  # joint_st2l
            math.atan2(D, A),  # joint_st1r
            math.atan2(C, A)   # joint_st2r
        ]

        # Publicar
        js = JointState()
        js.name = self.joint_names
        js.position = angles
        self.pub.publish(js)

def main():
    rclpy.init()
    node = ServoDirectionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
