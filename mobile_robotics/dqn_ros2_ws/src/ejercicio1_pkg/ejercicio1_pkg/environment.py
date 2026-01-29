"""
Environment Wrapper para DQN Navigation con Stage

Este módulo implementa un nodo ROS2 que actúa como interfaz entre
el agente DQN y el simulador Stage.

Funcionalidades:
- Suscripción a /base_scan (LiDAR) y /odom o /odom/sim (odometría)
- Publicación de comandos de velocidad en /cmd_vel
- Sistema de recompensas: +200 meta, -100 colisión, intermedias por progreso
- Reset de episodio usando servicio /reset_positions de Stage

NOTA CRÍTICA: Stage NO tiene /reset_world como Gazebo. 
Usamos /reset_positions para teletransportar al robot + nueva meta aleatoria.

Autor: Proyecto Académico DQN Navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import math
import random
from typing import Tuple, Optional

from .state_processor import StateProcessor


# ============================================================================
# PARÁMETROS DEL ENTORNO
# ============================================================================

# Límites del mapa (cave.world es aproximadamente 16x16 metros)
MAP_X_MIN = -6.0
MAP_X_MAX = 6.0
MAP_Y_MIN = -6.0
MAP_Y_MAX = 6.0

# Umbrales de distancia
COLLISION_THRESHOLD = 0.30    # Distancia para considerar colisión (metros)
GOAL_THRESHOLD = 0.50         # Distancia para considerar meta alcanzada (metros)

# ============================================================================
# SISTEMA DE RECOMPENSAS (según especificaciones del proyecto)
# ============================================================================
REWARD_GOAL = 200.0           # Recompensa: +200 por alcanzar objetivo
REWARD_COLLISION = -100.0     # Recompensa: -100 por colisión
REWARD_PROGRESS_SCALE = 5.0   # Escala para recompensas intermedias

# ============================================================================
# ESPACIO DE ACCIONES (5 acciones discretas)
# ============================================================================
# Cada acción es una tupla (linear_x, angular_z)
ACTIONS = {
    0: (0.3, 0.0),     # Avanzar recto
    1: (0.2, 0.3),     # Avanzar + rotar izquierda suave
    2: (0.2, -0.3),    # Avanzar + rotar derecha suave
    3: (0.1, 0.6),     # Rotar izquierda fuerte
    4: (0.1, -0.6),    # Rotar derecha fuerte
}
NUM_ACTIONS = len(ACTIONS)


class EnvironmentNode(Node):
    """
    Nodo ROS2 que actúa como wrapper del entorno Stage para DQN.
    
    Proporciona una interfaz tipo Gym con métodos:
    - step(action): Ejecuta acción, retorna (state, reward, done, info)
    - reset(): Reinicia episodio, retorna estado inicial
    """
    
    def __init__(self):
        super().__init__('dqn_environment')
        
        # Procesador de estados
        self.state_processor = StateProcessor()
        
        # Estado actual del entorno
        self.current_scan: Optional[LaserScan] = None
        self.current_odom: Optional[Odometry] = None
        self.previous_distance = float('inf')
        
        # Flags de estado
        self.data_ready = False
        self.episode_done = False
        self.collision = False
        self.goal_reached = False
        
        # QoS para sensores
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # ====================================================================
        # SUSCRIPCIONES
        # ====================================================================
        # LiDAR - En Stage el topic es /base_scan (no /scan como en Gazebo)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.scan_callback,
            sensor_qos
        )
        
        # Odometría - Usamos /odom/sim si está disponible (del reset wrapper)
        # Si no, usamos /odom directo
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom/sim',
            self.odom_callback,
            10
        )
        
        # ====================================================================
        # PUBLICACIONES
        # ====================================================================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ====================================================================
        # CLIENTE DE SERVICIO PARA RESET
        # Stage usa /reset_positions en lugar de /reset_world
        # ====================================================================
        self.reset_client = self.create_client(Empty, '/reset_sim')
        
        # Generar meta inicial
        self._generate_random_goal()
        
        self.get_logger().info('Environment Node initialized')
        self.get_logger().info(f'Action space: {NUM_ACTIONS} actions')
        self.get_logger().info(f'State size: {self.state_processor.get_state_size()}')
    
    def scan_callback(self, msg: LaserScan):
        """Callback para datos del LiDAR."""
        self.current_scan = msg
        self._check_data_ready()
    
    def odom_callback(self, msg: Odometry):
        """Callback para datos de odometría."""
        self.current_odom = msg
        self._check_data_ready()
    
    def _check_data_ready(self):
        """Verifica si tenemos todos los datos necesarios."""
        if self.current_scan is not None and self.current_odom is not None:
            self.data_ready = True
    
    def _generate_random_goal(self):
        """
        Genera una nueva posición objetivo aleatoria dentro del mapa.
        Evita generar metas muy cerca de los bordes o del robot.
        """
        margin = 1.0  # Margen desde los bordes
        
        while True:
            goal_x = random.uniform(MAP_X_MIN + margin, MAP_X_MAX - margin)
            goal_y = random.uniform(MAP_Y_MIN + margin, MAP_Y_MAX - margin)
            
            # Verificar que no esté muy cerca del robot
            if self.current_odom is not None:
                robot_x = self.current_odom.pose.pose.position.x
                robot_y = self.current_odom.pose.pose.position.y
                dist = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
                if dist > 2.0:  # Al menos 2 metros de distancia
                    break
            else:
                break
        
        self.state_processor.set_goal(goal_x, goal_y)
        self.get_logger().info(f'New goal: ({goal_x:.2f}, {goal_y:.2f})')
    
    def get_state(self) -> np.ndarray:
        """
        Obtiene el estado actual del entorno.
        
        Returns:
            Vector de estado de 22 dimensiones
        """
        if not self.data_ready:
            return np.zeros(self.state_processor.get_state_size())
        
        return self.state_processor.get_state(self.current_scan, self.current_odom)
    
    def get_state_size(self) -> int:
        """Retorna el tamaño del vector de estado."""
        return self.state_processor.get_state_size()
    
    def get_action_size(self) -> int:
        """Retorna el número de acciones disponibles."""
        return NUM_ACTIONS
    
    def execute_action(self, action: int):
        """
        Ejecuta una acción publicando comando de velocidad.
        
        Args:
            action: Índice de la acción (0 a NUM_ACTIONS-1)
        """
        if action not in ACTIONS:
            self.get_logger().warn(f'Invalid action: {action}')
            action = 0
        
        linear_x, angular_z = ACTIONS[action]
        
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        
        self.cmd_vel_pub.publish(cmd)
    
    def stop_robot(self):
        """Detiene el robot publicando velocidades cero."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def _calculate_reward(self) -> Tuple[float, bool, str]:
        """
        Calcula la recompensa basada en el estado actual.
        
        Implementa el sistema de recompensas especificado:
        - +200: Alcanzar objetivo (distancia < GOAL_THRESHOLD)
        - -100: Colisión (distancia mínima < COLLISION_THRESHOLD)
        - Intermedia: +5 * (distancia_anterior - distancia_actual)
        
        Returns:
            Tuple de (reward, done, info_string)
        """
        # Obtener distancias
        distance_to_goal = self.state_processor.get_distance_to_goal()
        min_obstacle_dist = self.state_processor.get_min_obstacle_distance()
        
        reward = 0.0
        done = False
        info = ""
        
        # Verificar COLISIÓN (-100 puntos)
        if min_obstacle_dist < COLLISION_THRESHOLD:
            reward = REWARD_COLLISION  # -100
            done = True
            self.collision = True
            info = "COLLISION"
            self.get_logger().warn(f'Collision detected! Min distance: {min_obstacle_dist:.2f}m')
        
        # Verificar META ALCANZADA (+200 puntos)
        elif distance_to_goal < GOAL_THRESHOLD:
            reward = REWARD_GOAL  # +200
            done = True
            self.goal_reached = True
            info = "GOAL_REACHED"
            self.get_logger().info(f'Goal reached! Distance: {distance_to_goal:.2f}m')
        
        # Recompensa INTERMEDIA (progreso hacia la meta)
        else:
            # Recompensa proporcional a la mejora en distancia
            progress = self.previous_distance - distance_to_goal
            reward = REWARD_PROGRESS_SCALE * progress  # +5 * (prev - curr)
            
            # Pequeña penalización por paso para incentivar eficiencia
            reward -= 0.1
            
            info = f"progress: {progress:.3f}"
        
        # Actualizar distancia anterior para siguiente paso
        self.previous_distance = distance_to_goal
        
        return reward, done, info
    
    def step(self, action: int) -> Tuple[np.ndarray, float, bool, str]:
        """
        Ejecuta un paso del entorno (interfaz tipo Gym).
        
        Args:
            action: Índice de la acción a ejecutar
            
        Returns:
            Tuple de (next_state, reward, done, info)
        """
        # Ejecutar acción
        self.execute_action(action)
        
        # Esperar un poco para que el robot se mueva
        # (esto se maneja en el loop principal con rate)
        
        # Procesar callbacks pendientes
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Obtener nuevo estado
        next_state = self.get_state()
        
        # Calcular recompensa
        reward, done, info = self._calculate_reward()
        
        if done:
            self.stop_robot()
        
        return next_state, reward, done, info
    
    def reset(self) -> np.ndarray:
        """
        Reinicia el episodio.
        
        NOTA CRÍTICA PARA STAGE:
        Stage no tiene /reset_world. Usamos /reset_positions para
        teletransportar al robot a la posición inicial, y generamos
        una nueva meta aleatoria.
        
        Returns:
            Estado inicial del nuevo episodio
        """
        self.get_logger().info('Resetting episode...')
        
        # Detener el robot
        self.stop_robot()
        
        # Llamar al servicio de reset de Stage
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            request = Empty.Request()
            future = self.reset_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self.get_logger().info('Reset service called successfully')
        else:
            self.get_logger().warn('Reset service not available')
        
        # Esperar un momento para que Stage procese el reset
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Reset flags
        self.episode_done = False
        self.collision = False
        self.goal_reached = False
        self.previous_distance = float('inf')
        
        # Generar nueva meta aleatoria
        self._generate_random_goal()
        
        # Esperar datos frescos
        self.data_ready = False
        while not self.data_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Inicializar distancia anterior
        self.previous_distance = self.state_processor.get_distance_to_goal()
        
        return self.get_state()
    
    def wait_for_data(self, timeout_sec: float = 5.0) -> bool:
        """
        Espera hasta que haya datos disponibles.
        
        Args:
            timeout_sec: Tiempo máximo de espera
            
        Returns:
            True si hay datos disponibles
        """
        import time
        start_time = time.time()
        
        while not self.data_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for sensor data')
                return False
        
        # Inicializar distancia anterior
        self.previous_distance = self.state_processor.get_distance_to_goal()
        
        return True
