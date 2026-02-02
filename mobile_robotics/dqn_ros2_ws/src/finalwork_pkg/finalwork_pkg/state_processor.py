"""
State Processor para DQN Navigation

Procesa datos sensoriales (LIDAR + odometría) para crear un vector de estado
normalizado que pueda ser usado por el agente DQN.

Estado resultante (22 dimensiones):
- [0:20]  Sectores LIDAR (distancia mínima normalizada por sector)
- [20]    Distancia al goal (normalizada)
- [21]    Ángulo al goal relativo al robot (normalizado a [-1, 1])

Autor: Proyecto DQN Navigation - finalwork_pkg
"""

import numpy as np
import math
from typing import Tuple, Optional
from dataclasses import dataclass

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# =============================================================================
# CONFIGURACIÓN
# =============================================================================
NUM_LIDAR_SECTORS = 20       # Número de sectores para discretizar LIDAR
LIDAR_MAX_RANGE = 50.0       # Rango máximo del LIDAR (metros)
GOAL_MAX_DISTANCE = 20.0     # Distancia máxima esperada al objetivo
LIDAR_FOV = 100.0            # Campo de visión LIDAR en grados (±60° desde el frente)


@dataclass
class RobotPose:
    """Representa la pose del robot en 2D."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class Goal:
    """Representa el objetivo de navegación."""
    x: float = 0.0
    y: float = 0.0


class StateProcessor:
    """
    Procesa datos sensoriales para crear el vector de estado del DQN.
    
    Attributes:
        num_sectors: Número de sectores para discretizar LIDAR
        state_size: Dimensión total del vector de estado
        robot_pose: Pose actual del robot
        goal: Objetivo actual
    """
    
    def __init__(self, num_sectors: int = NUM_LIDAR_SECTORS):
        """
        Inicializa el procesador de estados.
        
        Args:
            num_sectors: Número de sectores para discretizar el LIDAR
        """
        self.num_sectors = num_sectors
        self.state_size = num_sectors + 2  # Sectores + distancia + ángulo
        
        # Estado del robot y objetivo
        self.robot_pose = RobotPose()
        self.goal = Goal()
        
        # Último escaneo LIDAR procesado
        self._lidar_sectors = np.ones(num_sectors)
    
    def process_lidar(self, scan: LaserScan) -> np.ndarray:
        """
        Procesa solo el arco FRONTAL del LIDAR según LIDAR_FOV.
        
        Funciona con FOV de LIDAR variable (e.g., Stage Hokuyo = 270°).
        Usa angle_min y angle_max del scan para calcular correctamente el frente.
        
        Args:
            scan: Mensaje LaserScan de ROS2
            
        Returns:
            Array de num_sectors distancias normalizadas [0, 1] solo del frente
            - 0 = muy cerca (peligro)
            - 1 = lejos (seguro)
        """
        ranges = np.array(scan.ranges)
        num_readings = len(ranges)
        
        if num_readings == 0:
            return np.ones(self.num_sectors)
        
        # Limpiar datos inválidos
        ranges = np.where(np.isinf(ranges), LIDAR_MAX_RANGE, ranges)
        ranges = np.where(np.isnan(ranges), LIDAR_MAX_RANGE, ranges)
        ranges = np.clip(ranges, 0.0, LIDAR_MAX_RANGE)
        
        # Calcular ángulo por lectura
        # Stage Hokuyo: angle_min = -135°, angle_max = 135°, FOV_real = 270°
        angle_min = scan.angle_min  # radianes
        angle_max = scan.angle_max  # radianes
        angle_increment = (angle_max - angle_min) / num_readings
        
        # Encontrar índice que corresponde al frente (0 grados)
        # El índice del frente está donde angle = 0
        front_index = int(-angle_min / angle_increment)
        
        # Calular cuántas lecturas corresponden a nuestro FOV frontal deseado
        half_fov_rad = np.radians(LIDAR_FOV / 2.0)  # LIDAR_FOV = 120° → ±60°
        half_fov_readings = int(half_fov_rad / abs(angle_increment))
        
        # Extraer índices de arco frontal centrado en front_index
        start_idx = max(0, front_index - half_fov_readings)
        end_idx = min(num_readings, front_index + half_fov_readings + 1)
        
        frontal_ranges = ranges[start_idx:end_idx]
        num_frontal_readings = len(frontal_ranges)
        
        if num_frontal_readings == 0:
            return np.ones(self.num_sectors)
        
        # Dividir lecturas frontales en sectores
        readings_per_sector = max(1, num_frontal_readings // self.num_sectors)
        sectors = np.zeros(self.num_sectors)
        
        for i in range(self.num_sectors):
            start = i * readings_per_sector
            end = min(start + readings_per_sector, num_frontal_readings)
            
            sector_readings = frontal_ranges[start:end]
            if len(sector_readings) > 0:
                # Distancia MÍNIMA del sector (más conservador)
                sectors[i] = np.min(sector_readings)
            else:
                sectors[i] = LIDAR_MAX_RANGE
        
        # Normalizar al rango [0, 1]
        self._lidar_sectors = sectors / LIDAR_MAX_RANGE
        
        return self._lidar_sectors
    
    def update_robot_pose(self, odom: Odometry) -> RobotPose:
        """
        Actualiza la pose del robot desde mensaje de odometría.
        
        Args:
            odom: Mensaje Odometry de ROS2
            
        Returns:
            RobotPose actualizada
        """
        self.robot_pose.x = odom.pose.pose.position.x
        self.robot_pose.y = odom.pose.pose.position.y
        
        # Extraer yaw del quaternion
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_pose.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return self.robot_pose
    
    def set_goal(self, x: float, y: float):
        """
        Establece la posición del objetivo.
        
        Args:
            x: Coordenada X del objetivo
            y: Coordenada Y del objetivo
        """
        self.goal.x = x
        self.goal.y = y
    
    def get_goal_vector(self) -> Tuple[float, float]:
        """
        Calcula el vector del robot al objetivo.
        
        Returns:
            Tuple (dx, dy) hacia el objetivo
        """
        dx = self.goal.x - self.robot_pose.x
        dy = self.goal.y - self.robot_pose.y
        return dx, dy
    
    def get_goal_info(self) -> Tuple[float, float]:
        """
        Calcula distancia y ángulo al objetivo en coordenadas polares
        RELATIVAS al robot.
        
        Returns:
            Tuple de (distancia_normalizada, ángulo_normalizado)
            - distancia: [0, 1] donde 1 es GOAL_MAX_DISTANCE
            - ángulo: [-1, 1] donde 0 es frente al robot
        """
        dx, dy = self.get_goal_vector()
        
        # Distancia euclidiana
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Ángulo global al objetivo
        angle_to_goal = math.atan2(dy, dx)
        
        # Ángulo RELATIVO al robot
        relative_angle = angle_to_goal - self.robot_pose.yaw
        
        # Normalizar ángulo a [-π, π]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi
        
        # Normalizar valores
        normalized_distance = min(distance / GOAL_MAX_DISTANCE, 1.0)
        normalized_angle = relative_angle / math.pi  # [-1, 1]
        
        return normalized_distance, normalized_angle
    
    def get_distance_to_goal(self) -> float:
        """
        Calcula la distancia actual al objetivo (sin normalizar).
        
        Returns:
            Distancia en metros
        """
        dx, dy = self.get_goal_vector()
        return math.sqrt(dx * dx + dy * dy)
    
    def get_min_obstacle_distance(self) -> float:
        """
        Obtiene la distancia mínima a cualquier obstáculo (sin normalizar).
        
        Returns:
            Distancia mínima en metros
        """
        return np.min(self._lidar_sectors) * LIDAR_MAX_RANGE
    
    def get_min_frontal_distance(self, fov_degrees: float = 120.0) -> float:
        """
        Obtiene la distancia mínima solo a obstáculos FRONTALES.
        
        Útil para evitación, ya que solo obstáculos al frente son peligrosos
        cuando el robot avanza.
        
        Args:
            fov_degrees: Campo de visión frontal en grados (default: 120° = ±60°)
            
        Returns:
            Distancia mínima frontal en metros
        """
        # Calcular cuántos sectores corresponden al FOV frontal
        # Los sectores van de 0 (frente) a num_sectors (360°)
        # Sector 0 = frente, sector num_sectors/2 = atrás
        
        fov_ratio = fov_degrees / 360.0
        num_frontal_sectors = int(self.num_sectors * fov_ratio)
        
        # Tomar sectores frontales (centrados en sector 0)
        half_sectors = num_frontal_sectors // 2
        
        # Sectores frontales: desde -half hasta +half (wrap around)
        frontal_indices = []
        for i in range(-half_sectors, half_sectors + 1):
            idx = i % self.num_sectors
            frontal_indices.append(idx)
        
        frontal_distances = self._lidar_sectors[frontal_indices]
        return np.min(frontal_distances) * LIDAR_MAX_RANGE
    
    def get_state(self, 
                  scan: Optional[LaserScan] = None,
                  odom: Optional[Odometry] = None) -> np.ndarray:
        """
        Construye el vector de estado completo.
        
        Args:
            scan: Mensaje LaserScan opcional (si None, usa último procesado)
            odom: Mensaje Odometry opcional (si None, usa última pose)
            
        Returns:
            Vector de estado de dimensión state_size (22)
        """
        if scan is not None:
            self.process_lidar(scan)
        
        if odom is not None:
            self.update_robot_pose(odom)
        
        # Información del objetivo
        goal_distance, goal_angle = self.get_goal_info()
        
        # Construir vector de estado
        state = np.zeros(self.state_size, dtype=np.float32)
        state[:self.num_sectors] = self._lidar_sectors
        state[self.num_sectors] = goal_distance
        state[self.num_sectors + 1] = goal_angle
        
        return state
    
    def get_state_size(self) -> int:
        """Retorna la dimensión del vector de estado."""
        return self.state_size
