"""
Odom Reset Wrapper para Stage Simulator

Este nodo proporciona un servicio /reset_sim que:
1. Llama al servicio /reset_positions de Stage para teletransportar el robot
2. Corrige la odometría para que el robot parezca empezar desde (0,0,0)

NOTA CRÍTICA: Stage NO tiene /reset_world como Gazebo.
El servicio /reset_positions teletransporta al robot a su posición inicial
definida en el archivo .world.

El nodo suscribe a /odom y publica /odom/sim con la odometría corregida.

Autor: Proyecto Académico DQN Navigation (basado en reset_stage.py original)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Quaternion
import math


class OdomResetWrapper(Node):
    """
    Nodo wrapper que proporciona odometría corregida y servicio de reset.
    
    - Suscribe a: /odom (odometría original de Stage)
    - Publica en: /odom/sim (odometría corregida desde el último reset)
    - Servicio: /reset_sim (resetea el robot y la odometría)
    """
    
    def __init__(self):
        super().__init__('odom_reset_wrapper')

        # --- Configuration ---
        self.input_odom_topic = '/odom'
        self.output_odom_topic = '/odom/sim'
        # Stage usa /reset_positions (std_srvs/Empty)
        self.stage_reset_service = '/reset_positions' 
        
        # --- Internal State ---
        # Guardamos la pose raw en el momento del reset
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0
        self.reset_triggered = False

        # --- Subscribers & Publishers ---
        self.sub_odom = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odom_callback,
            10
        )
        self.pub_odom = self.create_publisher(
            Odometry,
            self.output_odom_topic,
            10
        )

        # --- Services ---
        # Servicio que otros nodos llaman para resetear
        self.srv_reset = self.create_service(
            Empty,
            'reset_sim',
            self.reset_sim_callback
        )

        # Cliente para hablar con el simulador Stage
        self.client_stage = self.create_client(Empty, self.stage_reset_service)

        self.get_logger().info("Odom Reset Wrapper Initialized")
        self.get_logger().info(f"  Input topic: {self.input_odom_topic}")
        self.get_logger().info(f"  Output topic: {self.output_odom_topic}")
        self.get_logger().info(f"  Reset service: /reset_sim -> {self.stage_reset_service}")

    def reset_sim_callback(self, request, response):
        """
        Callback del servicio /reset_sim.
        
        1. Llama al servicio de reset de Stage (teletransporta robot)
        2. Marca flag para actualizar el offset de odometría
        """
        self.get_logger().info("Reset requested: Teleporting robot and zeroing odom...")
        
        # 1. Llamar a Stage /reset_positions
        if self.client_stage.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.client_stage.call_async(req)
            # Esperar un momento para que Stage procese
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            self.get_logger().info("Stage reset service called successfully")
        else:
            self.get_logger().error(f"Service {self.stage_reset_service} not available!")
            return response

        # 2. Marcar para actualizar offset en el próximo callback de odom
        self.reset_triggered = True
        
        return response

    def odom_callback(self, msg: Odometry):
        """
        Callback de odometría.
        
        Transforma la odometría original para que parezca que el robot
        empieza desde (0,0,0) después de cada reset.
        """
        # Extraer datos raw
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        _, _, raw_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        # Si hubo reset, guardar los valores actuales como nuevo "cero"
        if self.reset_triggered:
            self.offset_x = raw_x
            self.offset_y = raw_y
            self.offset_yaw = raw_yaw
            self.reset_triggered = False
            self.get_logger().info(
                f"Odom offset updated: x={raw_x:.2f}, y={raw_y:.2f}, yaw={math.degrees(raw_yaw):.1f}°"
            )

        # --- Transformación ---
        # Calcular pose corregida (T_local = T_offset_inv * T_global)
        dx = raw_x - self.offset_x
        dy = raw_y - self.offset_y
        
        # Rotar el vector delta por el yaw negativo del offset
        cos_inv = math.cos(-self.offset_yaw)
        sin_inv = math.sin(-self.offset_yaw)
        
        sim_x = dx * cos_inv - dy * sin_inv
        sim_y = dx * sin_inv + dy * cos_inv
        sim_yaw = raw_yaw - self.offset_yaw

        # Normalizar yaw a [-pi, pi]
        sim_yaw = (sim_yaw + math.pi) % (2 * math.pi) - math.pi

        # --- Construir mensaje corregido ---
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.child_frame_id = msg.child_frame_id
        
        # Posición corregida
        new_msg.pose.pose.position.x = sim_x
        new_msg.pose.pose.position.y = sim_y
        new_msg.pose.pose.position.z = msg.pose.pose.position.z

        # Orientación corregida
        new_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, sim_yaw)

        # Twist (velocidad) - en frame del robot, no cambia
        new_msg.twist = msg.twist

        self.pub_odom.publish(new_msg)

    # --- Math Helpers ---
    def euler_from_quaternion(self, q):
        """
        Convierte quaternion a ángulos de Euler (roll, pitch, yaw).
        """
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convierte ángulos de Euler a quaternion.
        """
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = OdomResetWrapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
