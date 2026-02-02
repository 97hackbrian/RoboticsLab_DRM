"""
Environment Manager para DQN Navigation con Stage

Wrapper del entorno Stage que proporciona una interfaz tipo Gym.
Maneja la comunicación con ROS2 y el cálculo de recompensas.

Nueva aproximación para Reset (sin nodo wrapper separado):
- Usa /ground_truth para obtener posición global real del robot
- Mantiene offset interno para calcular posición relativa
- Llama a /reset_positions para teletransportar robot

Generación de Goals:
- Genera goals aleatorios en áreas libres del mapa cave.world
- Evita obstáculos conocidos

Autor: Proyecto DQN Navigation - finalwork_pkg
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
import time
from typing import Tuple, Optional, List
from dataclasses import dataclass

from .state_processor import StateProcessor


# =============================================================================
# CONFIGURACIÓN DEL MAPA - cave.world
# =============================================================================
# El mapa cave.world es 16x16 metros, centrado en origen
# Definimos zonas libres basadas en el bitmap del mapa

@dataclass
class FreeZone:
    """Representa una zona rectangular libre de obstáculos."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    
    def contains(self, x: float, y: float) -> bool:
        return (self.x_min <= x <= self.x_max and 
                self.y_min <= y <= self.y_max)
    
    def random_point(self) -> Tuple[float, float]:
        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)
        return x, y


# Zonas libres en cave.world (basado en análisis del bitmap)
FREE_ZONES: List[FreeZone] = [
    # Esquina inferior izquierda (donde inicia el robot)
    FreeZone(x_min=-7.0, x_max=-4.0, y_min=-7.0, y_max=-4.0),
    # Zona central-izquierda
    FreeZone(x_min=-7.0, x_max=-5.0, y_min=-3.0, y_max=0.0),
    # Zona lateral izquierda superior
    FreeZone(x_min=-7.0, x_max=-5.0, y_min=2.0, y_max=5.0),
    # Zona central
    FreeZone(x_min=-3.0, x_max=1.0, y_min=-3.0, y_max=1.0),
    # Zona derecha inferior
    FreeZone(x_min=3.0, x_max=6.0, y_min=-6.0, y_max=-3.0),
    # Zona derecha superior
    FreeZone(x_min=5.0, x_max=7.0, y_min=5.0, y_max=7.0),
]

# Posición inicial del robot en cave.world
ROBOT_INITIAL_X = -7.0
ROBOT_INITIAL_Y = -7.0
ROBOT_INITIAL_YAW = math.radians(45)  # 45 grados


# =============================================================================
# UMBRALES Y LÍMITES
# =============================================================================
COLLISION_THRESHOLD = 0.30      # Metros - distancia mínima para colisión
GOAL_THRESHOLD = 0.8            # Metros - distancia para considerar goal alcanzado
MAX_DISTANCE_FROM_GOAL = 20.0   # Metros - límite antes de terminar episodio (sincronizado con state normalization)
MAX_STEPS_PER_EPISODE = 400     # Pasos máximos por episodio (permite navegación larga)


# =============================================================================
# SISTEMA DE RECOMPENSAS
# =============================================================================
# IMPORTANTE: REWARD_GOAL debe estar balanceado 1:1 con |REWARD_COLLISION|
# para evitar sesgo hacia "no hacer nada". El agente debe arriesgar para alcanzar goals.

REWARD_GOAL = 200.0             # Recompensa por alcanzar objetivo (BALANCEADO con colisión)
REWARD_COLLISION = -200.0       # Penalización por colisión (balanceada 1:1 con goal)
REWARD_TIMEOUT = -14.0          # Penalización por timeout
REWARD_TOO_FAR = -30.0          # Penalización por alejarse demasiado
REWARD_STEP = -0.001            # Penalización por paso (muy pequeña)
REWARD_PROGRESS = 25.0          # Multiplicador de progreso (balanceado)
REWARD_HEADING = 1.0            # Bonus por orientación (pequeño)
REWARD_SAFETY = 0.05            # Bonus por mantener distancia segura (aumentado 100x)
REWARD_PROXIMITY_SCALE = 15.0   # Escala para penalización por proximidad
REWARD_DANGER_ZONE = -5.0       # Penalización por zona de peligro (muy cerca)


# =============================================================================
# ESPACIO DE ACCIONES - Favorece movimiento hacia adelante
# =============================================================================
# Acciones combinadas para evitar oscilación pura
ACTIONS = {
    0: (0.5, 0.0),      # Avanzar recto (acción principal)
    1: (0.45, 0.25),      # Avanzar + girar izquierda suave
    2: (0.45, -0.25),     # Avanzar + girar derecha suave
    3: (0.1, 0.65),      # Avanzar lento + girar izquierda fuerte
    4: (0.1, -0.65),     # Avanzar lento + girar derecha fuerte
}
NUM_ACTIONS = len(ACTIONS)

# Frecuencia de publicación de cmd_vel
# NOTA: No usar más de 100Hz para evitar sobrecargar Stage
CMD_VEL_PUBLISH_RATE = 10.0  # Hz - valor seguro para simulación acelerada


class EnvironmentManager(Node):
    """
    Wrapper del entorno Stage con interfaz tipo Gym.
    
    Métodos principales:
    - step(action): Ejecuta acción, retorna (state, reward, done, info)
    - reset(): Reinicia episodio, retorna estado inicial
    
    Nueva aproximación para odometría:
    - Suscribe a /ground_truth para posición global real
    - Mantiene offset interno para calcular posición relativa al reset
    - No requiere nodo wrapper separado
    """
    
    def __init__(self, goal_x: Optional[float] = None, goal_y: Optional[float] = None):
        """
        Inicializa el Environment Manager.
        
        Args:
            goal_x: Coordenada X del goal (None para aleatorio)
            goal_y: Coordenada Y del goal (None para aleatorio)
        """
        super().__init__('dqn_environment')
        
        # Goal fijo o aleatorio
        self.fixed_goal = goal_x is not None and goal_y is not None
        self.fixed_goal_x = goal_x
        self.fixed_goal_y = goal_y
        
        # Procesador de estados
        self.state_processor = StateProcessor()
        
        # Datos actuales de sensores
        self.current_scan: Optional[LaserScan] = None
        self.current_ground_truth: Optional[Odometry] = None
        
        # Offset para posición relativa (nueva aproximación)
        self.position_offset_x = 0.0
        self.position_offset_y = 0.0
        self.position_offset_yaw = 0.0
        self.reset_pending = False
        
        # Distancias anteriores para cálculo de progreso
        self.previous_distance_x = float('inf')
        self.previous_distance_y = float('inf')
        
        # Comando de velocidad actual
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # Flags de estado
        self.data_ready = False
        self.collision = False
        self.goal_reached = False
        self.steps_in_episode = 0
        
        # Historial de acciones para detectar oscilación
        self.last_action = -1
        self.direction_changes = 0  # Contador de cambios de dirección
        
        # QoS para sensores
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # -----------------------------------------------------------------
        # SUSCRIPCIONES
        # -----------------------------------------------------------------
        # LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/base_scan',
            self._scan_callback,
            sensor_qos
        )
        
        # Ground Truth (posición global real del robot)
        self.ground_truth_sub = self.create_subscription(
            Odometry,
            '/ground_truth',
            self._ground_truth_callback,
            10
        )
        
        # -----------------------------------------------------------------
        # PUBLICACIONES
        # -----------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer para publicar cmd_vel continuamente (evita watchdog timeout)
        timer_period = 1.0 / CMD_VEL_PUBLISH_RATE
        self.cmd_vel_timer = self.create_timer(timer_period, self._publish_cmd_vel)
        
        # -----------------------------------------------------------------
        # CLIENTE DE SERVICIO PARA RESET
        # -----------------------------------------------------------------
        self.reset_client = self.create_client(Empty, '/reset_positions')
        
        # Generar goal inicial
        self._generate_goal()
        
        self.get_logger().info('Environment Manager initialized')
        self.get_logger().info(f'Action space: {NUM_ACTIONS} actions')
        self.get_logger().info(f'State size: {self.state_processor.get_state_size()}')
    
    # =========================================================================
    # CALLBACKS
    # =========================================================================
    
    def _scan_callback(self, msg: LaserScan):
        """Callback para datos del LIDAR."""
        self.current_scan = msg
        self._check_data_ready()
    
    def _ground_truth_callback(self, msg: Odometry):
        """
        Callback para ground truth (posición global real).
        
        SIMPLIFICADO: Usamos coordenadas GLOBALES directamente.
        El goal también está en coordenadas globales, así que no hay problema.
        """
        self.current_ground_truth = msg
        
        # Usar posición global directamente (sin ajuste de offset)
        self.state_processor.update_robot_pose(msg)
        
        self._check_data_ready()
    
    def _check_data_ready(self):
        """Verifica si tenemos todos los datos necesarios."""
        if self.current_scan is not None and self.current_ground_truth is not None:
            self.data_ready = True
    
    # =========================================================================
    # MANEJO DE POSICIÓN (Nueva aproximación sin wrapper)
    # =========================================================================
    
    def _capture_position_offset(self, odom: Odometry):
        """
        Captura la posición actual como offset para el reset.
        
        Después de esto, las posiciones se calcularán relativas a este punto.
        """
        self.position_offset_x = odom.pose.pose.position.x
        self.position_offset_y = odom.pose.pose.position.y
        
        # Extraer yaw del quaternion
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.position_offset_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.get_logger().info(
            f'Position offset captured: x={self.position_offset_x:.2f}, '
            f'y={self.position_offset_y:.2f}, yaw={math.degrees(self.position_offset_yaw):.1f}°'
        )
    
    def _create_adjusted_odom(self, raw_odom: Odometry) -> Odometry:
        """
        Crea mensaje de odometría ajustado (relativo al offset).
        
        Args:
            raw_odom: Odometría global (ground_truth)
            
        Returns:
            Odometría ajustada (relativa al último reset)
        """
        # Posición global
        raw_x = raw_odom.pose.pose.position.x
        raw_y = raw_odom.pose.pose.position.y
        
        # Yaw global
        q = raw_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calcular delta desde offset
        dx = raw_x - self.position_offset_x
        dy = raw_y - self.position_offset_y
        
        # Rotar por el yaw inverso del offset
        cos_inv = math.cos(-self.position_offset_yaw)
        sin_inv = math.sin(-self.position_offset_yaw)
        
        adj_x = dx * cos_inv - dy * sin_inv
        adj_y = dx * sin_inv + dy * cos_inv
        adj_yaw = raw_yaw - self.position_offset_yaw
        
        # Normalizar yaw a [-π, π]
        adj_yaw = (adj_yaw + math.pi) % (2 * math.pi) - math.pi
        
        # Crear nuevo mensaje
        adjusted = Odometry()
        adjusted.header = raw_odom.header
        adjusted.child_frame_id = raw_odom.child_frame_id
        adjusted.pose.pose.position.x = adj_x
        adjusted.pose.pose.position.y = adj_y
        adjusted.pose.pose.position.z = raw_odom.pose.pose.position.z
        
        # Quaternion desde yaw ajustado
        adjusted.pose.pose.orientation.x = 0.0
        adjusted.pose.pose.orientation.y = 0.0
        adjusted.pose.pose.orientation.z = math.sin(adj_yaw / 2)
        adjusted.pose.pose.orientation.w = math.cos(adj_yaw / 2)
        
        # Twist no cambia (ya está en frame del robot)
        adjusted.twist = raw_odom.twist
        
        return adjusted
    
    # =========================================================================
    # GENERACIÓN DE GOALS (en coordenadas GLOBALES del mapa)
    # =========================================================================
    
    def _generate_goal(self):
        """
        Genera la posición del objetivo en coordenadas GLOBALES.
        
        El robot usa ground_truth que también está en coordenadas globales,
        así que la distancia se calcula correctamente.
        """
        if self.fixed_goal:
            goal_x = self.fixed_goal_x
            goal_y = self.fixed_goal_y
        else:
            goal_x, goal_y = self._generate_random_goal_in_free_zone()
        
        self.state_processor.set_goal(goal_x, goal_y)
        self.get_logger().info(f'Goal set (GLOBAL): ({goal_x:.2f}, {goal_y:.2f})')
    
    def _generate_random_goal_in_free_zone(self) -> Tuple[float, float]:
        """
        Genera un goal aleatorio en una zona libre del mapa.
        
        Evita generar goals muy cerca de la posición actual del robot.
        
        Returns:
            Tuple (x, y) del goal
        """
        max_attempts = 100
        
        for _ in range(max_attempts):
            # Seleccionar zona aleatoria
            zone = random.choice(FREE_ZONES)
            goal_x, goal_y = zone.random_point()
            
            # Verificar que no está muy cerca del robot
            robot_x = self.state_processor.robot_pose.x
            robot_y = self.state_processor.robot_pose.y
            dist = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            
            if dist > 2.0:  # Al menos 2 metros de distancia
                return goal_x, goal_y
        
        # Fallback: usar primera zona
        return FREE_ZONES[0].random_point()
    
    # =========================================================================
    # PUBLICACIÓN DE COMANDOS
    # =========================================================================
    
    def _publish_cmd_vel(self):
        """Callback del timer para publicar cmd_vel continuamente."""
        cmd = Twist()
        cmd.linear.x = self.current_linear_x
        cmd.angular.z = self.current_angular_z
        self.cmd_vel_pub.publish(cmd)
    
    def set_action(self, action: int):
        """
        Establece la acción actual.
        
        Args:
            action: Índice de la acción (0 a NUM_ACTIONS-1)
        """
        if action not in ACTIONS:
            self.get_logger().warn(f'Invalid action: {action}')
            action = 0
        
        self.current_linear_x, self.current_angular_z = ACTIONS[action]
    
    def stop_robot(self):
        """Detiene el robot."""
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
    
    # =========================================================================
    # CÁLCULO DE RECOMPENSAS
    # =========================================================================
    
    def _calculate_reward(self) -> Tuple[float, bool, str]:
        """
        Calcula la recompensa basada en el estado actual.
        
        Returns:
            Tuple de (reward, done, info_string)
        """
        distance_to_goal = self.state_processor.get_distance_to_goal()
        
        # Usar distancia FRONTAL para evitación (solo obstáculos al frente importan)
        min_frontal_dist = self.state_processor.get_min_frontal_distance(fov_degrees=120.0)
        
        # Mantener min_obstacle_dist para logging/info (360°)
        min_obstacle_dist = self.state_processor.get_min_obstacle_distance()
        
        # Componentes vectoriales para progreso por ejes
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        dist_x = abs(goal_dx)
        dist_y = abs(goal_dy)
        
        reward = 0.0
        done = False
        info = ""
        
        # ===== CONDICIONES TERMINALES =====
        
        # COLISIÓN (usar distancia frontal)
        if min_frontal_dist < COLLISION_THRESHOLD:
            reward = REWARD_COLLISION
            done = True
            self.collision = True
            info = "COLLISION"
            self.get_logger().warn(f'Collision! Min dist: {min_obstacle_dist:.2f}m')
        
        # GOAL ALCANZADO
        elif distance_to_goal < GOAL_THRESHOLD:
            reward = REWARD_GOAL
            done = True
            self.goal_reached = True
            info = "GOAL_REACHED"
            self.get_logger().info(f'Goal reached! Dist: {distance_to_goal:.2f}m')
        
        # DEMASIADO LEJOS
        elif distance_to_goal > MAX_DISTANCE_FROM_GOAL:
            reward = REWARD_TOO_FAR
            done = True
            info = "TOO_FAR_FROM_GOAL"
            self.get_logger().warn(f'Too far from goal! Dist: {distance_to_goal:.2f}m')
        
        # TIMEOUT
        elif self.steps_in_episode >= MAX_STEPS_PER_EPISODE:
            reward = REWARD_TIMEOUT
            done = True
            info = "TIMEOUT"
            self.get_logger().warn(f'Episode timeout after {self.steps_in_episode} steps')
        
        # ===== RECOMPENSAS INTERMEDIAS =====
        else:
            # Progreso euclidiano (distancia directa al goal)
            current_distance = self.state_processor.get_distance_to_goal()
            if hasattr(self, 'previous_distance'):
                progress = self.previous_distance - current_distance
                progress = np.clip(progress, -0.5, 0.5)
                reward += REWARD_PROGRESS * progress
                
                # Bonus por mantener distancia de seguridad MIENTRAS progresa
                if progress > 0 and min_obstacle_dist > 0.5:
                    reward += 0.01  # Pequeño bonus por navegar seguro
            self.previous_distance = current_distance
            
            # Tracking de oscilación (para logging, sin penalización directa)
            current_action_direction = 0  # recto
            if self.current_angular_z > 0.1:
                current_action_direction = 1  # izquierda
            elif self.current_angular_z < -0.1:
                current_action_direction = -1  # derecha
            
            if hasattr(self, 'last_direction'):
                if (self.last_direction == 1 and current_action_direction == -1) or \
                   (self.last_direction == -1 and current_action_direction == 1):
                    self.direction_changes += 1
                else:
                    # Reducir contador gradualmente
                    self.direction_changes = max(0, self.direction_changes - 0.5)
            self.last_direction = current_action_direction
            
            # Orientación (solo si estamos lejos Y no oscilando mucho)
            if distance_to_goal > 1.5 and self.direction_changes < 3:
                _, goal_angle = self.state_processor.get_goal_info()
                heading_bonus = 1.0 - abs(goal_angle)
                reward += REWARD_HEADING * heading_bonus
            
            # === SISTEMA DE EVITACIÓN DE OBSTÁCULOS ===
            # Penalización gradual por proximidad: aumenta exponencialmente al acercarse
            # SOLO considera obstáculos FRONTALES (±60°)
            DANGER_THRESHOLD = 0.4  # Muy cerca del umbral de colisión (0.30)
            SAFE_DISTANCE = 0.8     # Distancia considerada segura
            
            if min_frontal_dist < DANGER_THRESHOLD:
                # Zona de peligro: penalización que crece al acercarse
                # A 0.30m (colisión): penalty ~= -15
                # A 0.35m: penalty ~= -7.5
                # A 0.40m: penalty ~= 0
                proximity_penalty = REWARD_PROXIMITY_SCALE * (DANGER_THRESHOLD - min_frontal_dist)
                reward += REWARD_DANGER_ZONE  # Penalización base por estar en zona peligrosa
                reward -= proximity_penalty
            elif min_frontal_dist > SAFE_DISTANCE:
                # Zona segura: bonus por mantener distancia
                reward += REWARD_SAFETY
                
            # Costo por paso
            reward += REWARD_STEP
            
            info = f"dist:{current_distance:.2f} prog:{progress if 'progress' in dir() else 0:.3f} osc:{self.direction_changes}"
        
        # Actualizar distancias anteriores
        self.previous_distance_x = dist_x
        self.previous_distance_y = dist_y
        
        return reward, done, info
    
    # =========================================================================
    # INTERFAZ GYM
    # =========================================================================
    
    def step(self, action: int) -> Tuple[np.ndarray, float, bool, dict]:
        """
        Ejecuta un paso del entorno.
        
        Args:
            action: Índice de la acción a ejecutar
            
        Returns:
            Tuple de (next_state, reward, done, info)
        """
        self.steps_in_episode += 1
        
        # Establecer acción (el timer la publica continuamente)
        self.set_action(action)
        
        # Esperar para que el robot se mueva
        # NOTA: Con interval_sim=10ms, el simulador corre 10x más rápido
        # pero necesitamos dar tiempo real para que ROS procese callbacks
        step_duration = 0.05  # 50ms real = ~500ms simulado a 10x
        end_time = time.time() + step_duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Obtener nuevo estado
        next_state = self.get_state()
        
        # Calcular recompensa
        reward, done, info_str = self._calculate_reward()
        
        if done:
            self.stop_robot()
        
        info = {
            'info': info_str,
            'steps': self.steps_in_episode,
            'distance_to_goal': self.state_processor.get_distance_to_goal(),
            'min_obstacle_dist': self.state_processor.get_min_obstacle_distance()
        }
        
        return next_state, reward, done, info
    
    def reset(self) -> np.ndarray:
        """
        Reinicia el episodio.
        
        Llama al servicio /reset_positions de Stage y marca
        flag para capturar nuevo offset de posición.
        
        Returns:
            Estado inicial del nuevo episodio
        """
        self.get_logger().info('Resetting episode...')
        
        # Detener robot
        self.stop_robot()
        
        # Llamar al servicio de reset de Stage
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            request = Empty.Request()
            future = self.reset_client.call_async(request)
            
            # Esperar con spin para mantener timer activo
            timeout = 2.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
            
            if future.done():
                self.get_logger().info('Reset service called successfully')
            else:
                self.get_logger().warn('Reset service timeout')
        else:
            self.get_logger().warn('Reset service not available')
        
        # Esperar que Stage procese el reset
        wait_time = 0.5
        end_time = time.time() + wait_time
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Reset flags
        self.collision = False
        self.goal_reached = False
        self.steps_in_episode = 0
        self.previous_distance_x = float('inf')
        self.previous_distance_y = float('inf')
        self.previous_distance = float('inf')
        self.last_direction = 0
        self.direction_changes = 0
        
        # Generar nuevo goal (si no es fijo)
        self._generate_goal()
        
        # Esperar datos frescos
        self.data_ready = False
        timeout = 2.0
        start = time.time()
        while not self.data_ready and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Log de debug: verificar posición del robot y goal
        robot_x = self.state_processor.robot_pose.x
        robot_y = self.state_processor.robot_pose.y
        goal_x = self.state_processor.goal.x
        goal_y = self.state_processor.goal.y
        dist = self.state_processor.get_distance_to_goal()
        self.get_logger().info(
            f'After reset - Robot: ({robot_x:.2f}, {robot_y:.2f}), '
            f'Goal: ({goal_x:.2f}, {goal_y:.2f}), Dist: {dist:.2f}m'
        )
        
        # Inicializar distancias anteriores
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        self.previous_distance_x = abs(goal_dx)
        self.previous_distance_y = abs(goal_dy)
        
        return self.get_state()
    
    def get_state(self) -> np.ndarray:
        """Obtiene el estado actual del entorno."""
        if not self.data_ready:
            return np.zeros(self.state_processor.get_state_size())
        
        return self.state_processor.get_state(self.current_scan, None)
    
    def get_state_size(self) -> int:
        """Retorna el tamaño del vector de estado."""
        return self.state_processor.get_state_size()
    
    def get_action_size(self) -> int:
        """Retorna el número de acciones disponibles."""
        return NUM_ACTIONS
    
    def wait_for_data(self, timeout_sec: float = 5.0) -> bool:
        """
        Espera hasta que haya datos disponibles.
        
        Args:
            timeout_sec: Tiempo máximo de espera
            
        Returns:
            True si hay datos disponibles
        """
        start_time = time.time()
        
        while not self.data_ready:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Timeout waiting for sensor data')
                return False
        
        # Inicializar distancias
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        self.previous_distance_x = abs(goal_dx)
        self.previous_distance_y = abs(goal_dy)
        
        return True
