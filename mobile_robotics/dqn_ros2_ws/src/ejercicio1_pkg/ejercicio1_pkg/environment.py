"""
Environment Wrapper para DQN Navigation con Stage

Este módulo implementa un nodo ROS2 que actúa como interfaz entre
el agente DQN y el simulador Stage.

Funcionalidades:
- Suscripción a /base_scan (LiDAR) y /odom o /odom/sim (odometría)
- Publicación de comandos de velocidad en /cmd_vel CON TIMER
- Sistema de recompensas: +200 meta, -100 colisión, intermedias por progreso
- Reset de episodio usando servicio /reset_positions de Stage

NOTA CRÍTICA: Stage NO tiene /reset_world como Gazebo. 
Usamos /reset_positions para teletransportar al robot + nueva meta aleatoria.

NOTA: Usamos un TIMER para publicar cmd_vel continuamente y evitar
el watchdog timeout de Stage.

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
import time
from typing import Tuple, Optional

from .state_processor import StateProcessor


# ============================================================================
# PARÁMETROS DEL ENTORNO
# ============================================================================

# Límites del mapa (cave.world es aproximadamente 16x16 metros)
MAP_X_MIN = 7.0
MAP_X_MAX = 7.6
MAP_Y_MIN = 1.0
MAP_Y_MAX = 2.0

# Umbrales de distancia
# Umbrales
COLLISION_THRESHOLD = 0.30     # Metros
GOAL_THRESHOLD = 1.1666          # Metros
MAX_DISTANCE_FROM_GOAL = 10.0  # Metros

# Límite de pasos
MAX_STEPS_PER_EPISODE = 500    # Default 500

# ============================================================================
# SISTEMA DE RECOMPENSAS SIMPLIFICADO (Estabilidad)
# ============================================================================
REWARD_GOAL = 100.0           # Meta (Valor alto pero controlado)
REWARD_COLLISION = -100.0     # Colisión
REWARD_TIMEOUT = -10.0        # Timeout
REWARD_TOO_FAR = -20.0        # Alejarse demasiado

# Recompensas densas (pequeños valores normalizados)
REWARD_STEP = -0.05           # Penalización por paso (incentiva rapidez)
REWARD_PROGRESS = 50.0         # Multiplicador de progreso (acercarse)
REWARD_HEADING = 0.1          # Bonus por orientación correcta
REWARD_SAFETY = 0.01           # Bonus por seguridad (>1m obstáculos)

# Nota: Eliminamos "momentum" y "distracción" porque introducen ruido/inestabilidad

# ============================================================================
# ESPACIO DE ACCIONES (5 acciones básicas)
# Simplificado para facilitar aprendizaje
# ============================================================================
ACTIONS = {
    0: (0.5, 0.0),     # Avanzar
    1: (0.0, 0.2),     # Giro Izquierda (en sitio)
    2: (0.0, -0.2),    # Giro Derecha (en sitio)
}
NUM_ACTIONS = len(ACTIONS)

# Frecuencia de publicación de cmd_vel (Hz) - suficiente para evitar watchdog
CMD_VEL_PUBLISH_RATE = 30.0  # 30Hz es suficiente, 190Hz era excesivo


class EnvironmentNode(Node):
    """
    Nodo ROS2 que actúa como wrapper del entorno Stage para DQN.
    
    Proporciona una interfaz tipo Gym con métodos:
    - step(action): Ejecuta acción, retorna (state, reward, done, info)
    - reset(): Reinicia episodio, retorna estado inicial
    
    IMPORTANTE: Usa un timer para publicar cmd_vel continuamente
    y evitar el watchdog timeout de Stage.
    """
    
    def __init__(self):
        super().__init__('dqn_environment')
        
        # Procesador de estados
        self.state_processor = StateProcessor()
        
        # Estado actual del entorno
        self.current_scan: Optional[LaserScan] = None
        self.current_odom: Optional[Odometry] = None
        self.previous_distance_x = float('inf')
        self.previous_distance_y = float('inf')
        
        # Comando de velocidad actual (se publica continuamente)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        
        # Flags de estado
        self.data_ready = False
        self.episode_done = False
        self.collision = False
        self.goal_reached = False
        self.is_resetting = False
        
        # Contador de pasos del episodio
        self.steps_in_episode = 0
        
        # ====================================================================
        # SISTEMA DE MEMORIA DE TRAYECTORIAS
        # Guarda las rutas que se acercan al goal para análisis/replay
        # ====================================================================
        self.current_trajectory = []  # Trayectoria del episodio actual
        self.successful_trajectories = []  # Lista de trayectorias exitosas
        self.best_trajectory = None  # Mejor trayectoria (más corta al goal)
        self.best_trajectory_steps = float('inf')  # Pasos de la mejor ruta
        
        # ====================================================================
        # SISTEMA DE MEMORIA DE TRAYECTORIAS
        # ====================================================================
        self.current_trajectory = [] 
        self.successful_trajectories = []
        self.best_trajectory = None
        self.best_trajectory_steps = float('inf')
        
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
        # TIMER PARA PUBLICAR CMD_VEL CONTINUAMENTE
        # Esto evita el watchdog timeout de Stage
        # ====================================================================
        timer_period = 1.0 / CMD_VEL_PUBLISH_RATE  # 50ms por defecto
        self.cmd_vel_timer = self.create_timer(timer_period, self._publish_cmd_vel)
        
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
        self.get_logger().info(f'Cmd_vel publish rate: {CMD_VEL_PUBLISH_RATE} Hz')
    
    def _publish_cmd_vel(self):
        """
        Callback del timer para publicar cmd_vel continuamente.
        Esto evita el watchdog timeout de Stage.
        """
        cmd = Twist()
        cmd.linear.x = self.current_linear_x
        cmd.angular.z = self.current_angular_z
        self.cmd_vel_pub.publish(cmd)
    
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
        margin = 0.05  # Margen pequeño para áreas reducidas
        
        while True:
            # Asegurar que min < max incluso con margen
            x_min = MAP_X_MIN + margin
            x_max = max(x_min + 0.1, MAP_X_MAX - margin)
            
            y_min = MAP_Y_MIN + margin
            y_max = max(y_min + 0.1, MAP_Y_MAX - margin)
            
            goal_x = random.uniform(x_min, x_max)
            goal_y = random.uniform(y_min, y_max)
            
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
    
    def set_action(self, action: int):
        """
        Establece la acción actual (se publicará continuamente por el timer).
        
        Args:
            action: Índice de la acción (0 a NUM_ACTIONS-1)
        """
        if action not in ACTIONS:
            self.get_logger().warn(f'Invalid action: {action}')
            action = 0
        
        self.current_linear_x, self.current_angular_z = ACTIONS[action]
    
    def stop_robot(self):
        """Detiene el robot estableciendo velocidades cero."""
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
    
    def _calculate_reward(self) -> Tuple[float, bool, str]:
        """
        Calcula la recompensa basada en el estado actual.
        
        Sistema de recompensas mejorado:
        - +200: Alcanzar objetivo (distancia < GOAL_THRESHOLD)
        - -100: Colisión (distancia mínima < COLLISION_THRESHOLD)
        - -50: Alejarse demasiado del objetivo
        - -20: Timeout (exceder MAX_STEPS_PER_EPISODE)
        - Shaped rewards: progreso, orientación, seguridad
        
        Returns:
            Tuple de (reward, done, info_string)
        """
        # Obtener distancias Euclidianas (para terminal conditions)
        distance_to_goal = self.state_processor.get_distance_to_goal()
        min_obstacle_dist = self.state_processor.get_min_obstacle_distance()
        
        # Obtener componentes vectoriales (para progreso por ejes)
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        dist_x = abs(goal_dx)
        dist_y = abs(goal_dy)
        
        reward = 0.0
        done = False
        info = ""
        
        # ================================================================
        # CONDICIONES TERMINALES
        # ================================================================
        
        # Verificar COLISIÓN (-100 puntos)
        if min_obstacle_dist < COLLISION_THRESHOLD:
            reward = REWARD_COLLISION  # -100
            done = True
            self.collision = True
            info = "COLLISION"
            self.get_logger().warn(f'Collision! Min dist: {min_obstacle_dist:.2f}m')
        
        # Verificar META ALCANZADA (+200 puntos)
        elif distance_to_goal < GOAL_THRESHOLD:
            reward = REWARD_GOAL  # +200
            done = True
            self.goal_reached = True
            info = "GOAL_REACHED"
            self.get_logger().info(f'Goal reached! Dist: {distance_to_goal:.2f}m')
        
        # Verificar demasiado LEJOS del objetivo (-50 puntos)
        elif distance_to_goal > MAX_DISTANCE_FROM_GOAL:
            reward = REWARD_TOO_FAR  # -50
            done = True
            info = "TOO_FAR_FROM_GOAL"
            self.get_logger().warn(f'Too far from goal! Dist: {distance_to_goal:.2f}m')
        
        # Verificar TIMEOUT (-20 puntos)
        elif self.steps_in_episode >= MAX_STEPS_PER_EPISODE:
            reward = REWARD_TIMEOUT  # -20
            done = True
            info = "TIMEOUT"
            self.get_logger().warn(f'Episode timeout after {self.steps_in_episode} steps')
        
        # ================================================================
        # RECOMPENSAS INTERMEDIAS (Shaped Rewards)
        # ================================================================
        else:
            # 1. Recompensa por PROGRESO (Component-wise)
            # Recompensa si reduce error en X y si reduce error en Y
            prog_x = self.previous_distance_x - dist_x
            prog_y = self.previous_distance_y - dist_y
            
            # Limitar spikes
            prog_x = np.clip(prog_x, -0.5, 0.5)
            prog_y = np.clip(prog_y, -0.5, 0.5)
            
            # Sumar progresos (si te acercas en ambos, ganas más)
            total_progress = prog_x + prog_y
            
            reward += REWARD_PROGRESS * total_progress
            
            # 2. Recompensa por ORIENTACIÓN
            # Solo aplicamos si estamos a cierta distancia para evitar inestabilidad (singularidad)
            if distance_to_goal > 1.0:
                _, goal_angle = self.state_processor.get_goal_info()
                heading_bonus = 1.0 - abs(goal_angle)
                reward += REWARD_HEADING * heading_bonus
            
            # 3. Recompensa por SEGURIDAD
            # Solo premiar si está holgadamente seguro
            if min_obstacle_dist > 0.8:
                reward += REWARD_SAFETY
            elif min_obstacle_dist < 0.4:
                reward -= 0.2
            
            # 4. Costo por paso
            reward += REWARD_STEP
            
            # Info string con detalles de ejes
            info = f"dX:{dist_x:.2f} dY:{dist_y:.2f} progX:{prog_x:.3f} progY:{prog_y:.3f}"
        
        # Actualizar distancias anteriores para siguiente paso
        self.previous_distance_x = dist_x
        self.previous_distance_y = dist_y
        
        return reward, done, info
    
    def step(self, action: int) -> Tuple[np.ndarray, float, bool, str]:
        """
        Ejecuta un paso del entorno (interfaz tipo Gym).
        
        Args:
            action: Índice de la acción a ejecutar
            
        Returns:
            Tuple de (next_state, reward, done, info)
        """
        # Incrementar contador de pasos
        self.steps_in_episode += 1
        
        # Establecer acción (el timer la publicará continuamente)
        self.set_action(action)
        
        # Esperar un tiempo para que el robot se mueva y recibir datos
        # Usamos spin para procesar callbacks mientras esperamos
        step_duration = 0.1  # 100ms por step
        end_time = time.time() + step_duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # Obtener nuevo estado
        next_state = self.get_state()
        
        # Calcular recompensa
        reward, done, info = self._calculate_reward()
        
        # ================================================================
        # GRABAR POSICIÓN EN LA TRAYECTORIA
        # ================================================================
        if self.current_odom is not None:
            trajectory_point = {
                'step': self.steps_in_episode,
                'x': self.current_odom.pose.pose.position.x,
                'y': self.current_odom.pose.pose.position.y,
                'action': action,
                'reward': reward,
                'distance_to_goal': self.state_processor.get_distance_to_goal()
            }
            self.current_trajectory.append(trajectory_point)
        
        # Guardar trayectoria si llegó al goal
        if done and self.goal_reached:
            self._save_successful_trajectory()
        
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
        self.is_resetting = True
        self.get_logger().info('Resetting episode...')
        
        # Detener el robot
        self.stop_robot()
        
        # Llamar al servicio de reset de Stage
        if self.reset_client.wait_for_service(timeout_sec=2.0):
            request = Empty.Request()
            future = self.reset_client.call_async(request)
            
            # Esperar con spin para mantener el timer activo
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
        
        # Esperar un momento para que Stage procese el reset
        # Seguimos haciendo spin para que el timer publique
        wait_time = 0.5
        end_time = time.time() + wait_time
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Reset flags
        self.episode_done = False
        self.collision = False
        self.goal_reached = False
        self.previous_distance = float('inf')
        self.steps_in_episode = 0  # Reset step counter
        self.current_trajectory = []  # Limpiar trayectoria del episodio
        
        # Generar nueva meta aleatoria
        self._generate_random_goal()
        
        # Esperar datos frescos
        self.data_ready = False
        timeout = 2.0
        start = time.time()
        while not self.data_ready and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Inicializar distancia anterior separada por ejes
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        self.previous_distance_x = abs(goal_dx)
        self.previous_distance_y = abs(goal_dy)
        
        self.is_resetting = False
        return self.get_state()
    
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
        
        # Inicializar distancia anterior separada por ejes
        goal_dx, goal_dy = self.state_processor.get_goal_vector()
        self.previous_distance_x = abs(goal_dx)
        self.previous_distance_y = abs(goal_dy)
        
        return True
    
    # ========================================================================
    # MÉTODOS DE GESTIÓN DE TRAYECTORIAS
    # ========================================================================
    
    def _save_successful_trajectory(self):
        """
        Guarda la trayectoria actual como exitosa si llegó al goal.
        También actualiza la mejor trayectoria si esta es más corta.
        """
        if not self.current_trajectory:
            return
        
        trajectory_data = {
            'steps': len(self.current_trajectory),
            'goal_x': self.state_processor.goal_x,
            'goal_y': self.state_processor.goal_y,
            'trajectory': self.current_trajectory.copy()
        }
        
        self.successful_trajectories.append(trajectory_data)
        
        # Actualizar mejor trayectoria si esta es más corta
        if len(self.current_trajectory) < self.best_trajectory_steps:
            self.best_trajectory = trajectory_data
            self.best_trajectory_steps = len(self.current_trajectory)
            self.get_logger().info(f'Nueva mejor trayectoria! {self.best_trajectory_steps} pasos')
        
        self.get_logger().info(f'Trayectoria guardada ({len(self.current_trajectory)} pasos). '
                               f'Total exitosas: {len(self.successful_trajectories)}')
    
    def get_successful_trajectories(self) -> list:
        """
        Retorna todas las trayectorias exitosas guardadas.
        
        Returns:
            Lista de diccionarios con datos de trayectorias
        """
        return self.successful_trajectories
    
    def get_best_trajectory(self) -> Optional[dict]:
        """
        Retorna la mejor trayectoria (más corta) que llegó al goal.
        
        Returns:
            Diccionario con la mejor trayectoria o None
        """
        return self.best_trajectory
    
    def save_trajectories_to_file(self, filepath: str):
        """
        Guarda todas las trayectorias exitosas en un archivo.
        
        Args:
            filepath: Ruta del archivo (ej: 'trajectories.pkl')
        """
        import pickle
        save_data = {
            'successful_trajectories': self.successful_trajectories,
            'best_trajectory': self.best_trajectory,
            'best_steps': self.best_trajectory_steps
        }
        with open(filepath, 'wb') as f:
            pickle.dump(save_data, f)
        self.get_logger().info(f'Trayectorias guardadas en {filepath}')
    
    def load_trajectories_from_file(self, filepath: str):
        """
        Carga trayectorias desde un archivo.
        
        Args:
            filepath: Ruta del archivo pickle
        """
        import pickle
        with open(filepath, 'rb') as f:
            save_data = pickle.load(f)
        
        self.successful_trajectories = save_data['successful_trajectories']
        self.best_trajectory = save_data['best_trajectory']
        self.best_trajectory_steps = save_data['best_steps']
        self.get_logger().info(f'Cargadas {len(self.successful_trajectories)} trayectorias')
    
    def get_trajectory_stats(self) -> dict:
        """
        Retorna estadísticas de las trayectorias guardadas.
        
        Returns:
            Diccionario con estadísticas
        """
        if not self.successful_trajectories:
            return {'total': 0, 'best_steps': None, 'avg_steps': None}
        
        steps_list = [t['steps'] for t in self.successful_trajectories]
        return {
            'total': len(self.successful_trajectories),
            'best_steps': self.best_trajectory_steps,
            'avg_steps': sum(steps_list) / len(steps_list),
            'min_steps': min(steps_list),
            'max_steps': max(steps_list)
        }
