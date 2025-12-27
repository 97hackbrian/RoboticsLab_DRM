#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
import sys

# Importa tus librerías del tanque
sys.path.append('tankbot/')
from motor_tiva_controller.libs.tiva import *


class MotorTivaController(Node):
    def __init__(self):
        super().__init__('motor_tiva_controller')

        # --- Inicializar comunicación con la Tiva ---
        self.get_logger().info('Inicializando comunicación con Tiva...')
        self.tiva = InitSerial(baud=9600)
        self.motors = Motors(serial_instance=self.tiva)
        self.gripper = Gripper(serial_instance=self.tiva)
        self.mosfets = Mosfets(serial_instance=self.tiva)
        self.leds = LedControl(serial_instance=self.tiva)

        # --- Configuración inicial ---
        self.leds.init_system(cam=0)
        self.mosfets.activate_mosfets(1, 1, 1, 1, 1, 1)

        # --- Suscripción al tópico /cmd_vel ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription

        # Parámetros de control
        self.max_speed = 35.0  # velocidad máxima motores (ajusta según tu escala)

    def listener_callback(self, msg: Twist):
        linear = msg.linear.x     # avance (m/s)
        angular = msg.angular.z   # giro (rad/s)

        # --- Conversión a velocidades de motor ---
        left_speed  = linear * self.max_speed - angular * (self.max_speed / 2)
        right_speed = linear * self.max_speed + angular * (self.max_speed / 2)

        # Saturar valores
        left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
        right_speed = max(min(right_speed, self.max_speed), -self.max_speed)

        self.get_logger().info(f"cmd_vel -> L:{left_speed:.2f}  R:{right_speed:.2f}")

        # --- Enviar velocidades a los motores ---
        self.motors.move(int(right_speed),int(left_speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorTivaController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

