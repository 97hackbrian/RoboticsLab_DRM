import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import math

class Evasor(Node):
    def __init__(self):
        super().__init__('evasor')
        # Suscriptor a la odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # Publicador de velocidad
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.start_position = None
        self.target_distance = 2.0  # Distancia objetivo en metros
        self.stopped = False
        
        self.get_logger().info('Nodo Evasor iniciado. Esperando odometría...')

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        
        # Guardar la posición inicial si es la primera vez
        if self.start_position is None:
            self.start_position = current_position
            self.get_logger().info(f'Posición inicial guardada: x={current_position.x:.2f}, y={current_position.y:.2f}')
            return

        # Calcular distancia euclidiana recorrida
        dx = current_position.x - self.start_position.x
        dy = current_position.y - self.start_position.y
        distance_traveled = math.sqrt(dx**2 + dy**2)

        cmd = TwistStamped()

        if distance_traveled < self.target_distance and not self.stopped:
            # Mover hacia adelante
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = 10.5  # Velocidad lineal en m/s
            
            self.get_logger().info(f'Avanzando... Distancia: {distance_traveled:.2f}/{self.target_distance} m', throttle_duration_sec=1.0)
        else:
            # Detener el robot
            cmd.twist.linear.x = 0.0
            
            if not self.stopped:
                self.get_logger().info(f'Distancia objetivo alcanzada ({distance_traveled:.2f} m). Deteniendo robot.')
                self.stopped = True
            
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Evasor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
