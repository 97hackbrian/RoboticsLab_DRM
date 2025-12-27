#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import math


class MotorJoystickController(Node):
    def __init__(self):
        super().__init__('motor_joystick_controller')

        # Publicador /cmd_vel estándar
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)

        self.twist = Twist()
        self.threshold = 0.1  # zona muerta para evitar ruido

        # Inicializa pygame y el joystick
        pygame.init()
        pygame.joystick.init()
        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.get_logger().info(f"Joystick detectado: {self.joystick.get_name()}")
        except pygame.error:
            self.get_logger().error("No se detectó ningún joystick conectado.")
            raise SystemExit

        self.get_logger().info("Nodo MotorJoystickController inicializado")

    def publish_twist(self):
        """Publica el mensaje Twist actual."""
        # Publica solo si hay movimiento
        if abs(self.twist.linear.x) > self.threshold or abs(self.twist.angular.z) > self.threshold:
            self.cmd_vel_pub.publish(self.twist)
        else:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)

    def update_controller(self):
        """Lee el estado del joystick y actualiza cmd_vel."""
        pygame.event.pump()

        # Ejes del joystick izquierdo (adelante/atrás y giro)
        axis_y = -self.joystick.get_axis(1)  # adelante (+)
        axis_x = -self.joystick.get_axis(0)   # giro (+ derecha)

        # Escalamos la velocidad
        max_linear = 0.55    # m/s
        max_angular = 1.3   # rad/s

        # Aplicar zona muerta
        self.twist.linear.x = axis_y * max_linear if abs(axis_y) > self.threshold else 0.0
        self.twist.angular.z = axis_x * max_angular if abs(axis_x) > self.threshold else 0.0

        # Log opcional
        self.get_logger().info(f"Joystick → linear: {self.twist.linear.x:.2f}, angular: {self.twist.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorJoystickController()

    try:
        while rclpy.ok():
            node.update_controller()
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()

