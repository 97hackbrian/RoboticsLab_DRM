"""
State Processor para DQN Navigation

Este módulo procesa los datos crudos del sensor LiDAR y la información
del objetivo para crear un vector de estado normalizado.

Procesamiento:
1. LiDAR 360 mediciones → 20 sectores (distancia mínima por sector)
2. Normalización de distancias al rango [0, 1]
3. Información del objetivo en coordenadas polares (distancia, ángulo)

Estado final: 22 dimensiones
- 20 sectores de LiDAR normalizados
- 1 distancia al objetivo (normalizada)
- 1 ángulo al objetivo (normalizado a [-1, 1])

Autor: Proyecto Académico DQN Navigation
"""

import numpy as np
import math
from typing import Tuple, Optional
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# ============================================================================
# PARÁMETROS DE PROCESAMIENTO
# ============================================================================
NUM_LIDAR_SECTORS = 20       # Número de sectores para discretizar LiDAR
LIDAR_MAX_RANGE = 10.0       # Rango máximo del LiDAR (metros)
GOAL_MAX_DISTANCE = 15.0     # Distancia máxima esperada al objetivo


class StateProcessor:
    """
    Procesa datos sensoriales para crear el vector de estado del DQN.
    
    El estado resultante es un vector de 22 dimensiones:
    - [0:20] Sectores LiDAR (distancias mínimas normalizadas)
    - [20]   Distancia al objetivo (normalizada)
    - [21]   Ángulo al objetivo (normalizado a [-1, 1])
    """
    
    def __init__(self, num_sectors: int = NUM_LIDAR_SECTORS):
        """
        Inicializa el procesador de estados.
        
        Args:
            num_sectors: Número de sectores para discretizar el LiDAR
        """
        self.num_sectors = num_sectors
        self.state_size = num_sectors + 2  # Sectores + distancia + ángulo
        
        # Última posición y orientación del robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Posición del objetivo
        self.goal_x = 0.0
        self.goal_y = 0.0
        
        # Último escaneo LiDAR procesado
        self.lidar_sectors = np.ones(num_sectors)  # Inicializado a máxima distancia
    
    def process_lidar(self, scan: LaserScan) -> np.ndarray:
        """
        Procesa un escaneo LiDAR de 360 grados en sectores discretos.
        
        Para cada sector, toma la DISTANCIA MÍNIMA como representación.
        Esto es crucial para detectar obstáculos cercanos que podrían
        causar colisiones.
        
        Args:
            scan: Mensaje LaserScan de ROS2
            
        Returns:
            Array de num_sectors distancias normalizadas [0, 1]
        """
        ranges = np.array(scan.ranges)
        num_readings = len(ranges)
        
        if num_readings == 0:
            return np.ones(self.num_sectors)
        
        # Reemplazar infinitos y NaN por el rango máximo
        ranges = np.where(np.isinf(ranges), LIDAR_MAX_RANGE, ranges)
        ranges = np.where(np.isnan(ranges), LIDAR_MAX_RANGE, ranges)
        
        # Limitar al rango máximo
        ranges = np.clip(ranges, 0, LIDAR_MAX_RANGE)
        
        # Dividir las lecturas en sectores
        readings_per_sector = num_readings // self.num_sectors
        sectors = np.zeros(self.num_sectors)
        
        for i in range(self.num_sectors):
            start_idx = i * readings_per_sector
            end_idx = start_idx + readings_per_sector
            
            if end_idx > num_readings:
                end_idx = num_readings
            
            # Tomar la distancia MÍNIMA del sector (más conservador para evitar colisiones)
            sector_readings = ranges[start_idx:end_idx]
            if len(sector_readings) > 0:
                sectors[i] = np.min(sector_readings)
            else:
                sectors[i] = LIDAR_MAX_RANGE
        
        # Normalizar al rango [0, 1]
        # 0 = muy cerca (peligro), 1 = lejos (seguro)
        self.lidar_sectors = sectors / LIDAR_MAX_RANGE
        
        return self.lidar_sectors
    
    def update_robot_pose(self, odom: Odometry):
        """
        Actualiza la posición y orientación del robot desde odometría.
        
        Args:
            odom: Mensaje Odometry de ROS2
        """
        self.robot_x = odom.pose.pose.position.x
        self.robot_y = odom.pose.pose.position.y
        
        # Extraer yaw del quaternion
        q = odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def set_goal(self, x: float, y: float):
        """
        Establece la posición del objetivo.
        
        Args:
            x: Coordenada X del objetivo
            y: Coordenada Y del objetivo
        """
        self.goal_x = x
        self.goal_y = y
    
    def get_goal_info(self) -> Tuple[float, float]:
        """
        Calcula la distancia y ángulo al objetivo en coordenadas polares
        RELATIVAS al robot.
        
        Returns:
            Tuple de (distancia_normalizada, ángulo_normalizado)
            - distancia: [0, 1] donde 1 es GOAL_MAX_DISTANCE
            - ángulo: [-1, 1] donde 0 es frente al robot
        """
        # Vector del robot al objetivo
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        
        # Distancia euclidiana
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Ángulo global al objetivo
        angle_to_goal = math.atan2(dy, dx)
        
        # Ángulo RELATIVO al robot (diferencia con orientación del robot)
        relative_angle = angle_to_goal - self.robot_yaw
        
        # Normalizar ángulo a [-π, π]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi
        
        # Normalizar distancia a [0, 1]
        normalized_distance = min(distance / GOAL_MAX_DISTANCE, 1.0)
        
        # Normalizar ángulo a [-1, 1]
        normalized_angle = relative_angle / math.pi
        
        return normalized_distance, normalized_angle
    
    def get_distance_to_goal(self) -> float:
        """
        Calcula la distancia actual al objetivo (sin normalizar).
        Útil para calcular recompensas intermedias.
        
        Returns:
            Distancia en metros
        """
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        return math.sqrt(dx * dx + dy * dy)
    
    def get_min_obstacle_distance(self) -> float:
        """
        Obtiene la distancia mínima a cualquier obstáculo (sin normalizar).
        Útil para detectar colisiones.
        
        Returns:
            Distancia mínima en metros
        """
        return np.min(self.lidar_sectors) * LIDAR_MAX_RANGE
    
    def get_state(self, scan: Optional[LaserScan] = None, 
                  odom: Optional[Odometry] = None) -> np.ndarray:
        """
        Construye el vector de estado completo.
        
        Si se proporcionan scan y odom, primero los procesa.
        Luego combina los sectores LiDAR con la información del objetivo.
        
        Args:
            scan: Mensaje LaserScan opcional
            odom: Mensaje Odometry opcional
            
        Returns:
            Vector de estado de dimensión state_size (22)
        """
        if scan is not None:
            self.process_lidar(scan)
        
        if odom is not None:
            self.update_robot_pose(odom)
        
        # Obtener información del objetivo
        goal_distance, goal_angle = self.get_goal_info()
        
        # Construir vector de estado
        # [sectores LiDAR (20), distancia objetivo (1), ángulo objetivo (1)]
        state = np.zeros(self.state_size)
        state[:self.num_sectors] = self.lidar_sectors
        state[self.num_sectors] = goal_distance
        state[self.num_sectors + 1] = goal_angle
        
        return state
    
    def get_state_size(self) -> int:
        """
        Retorna el tamaño del vector de estado.
        
        Returns:
            Dimensión del estado (22 por defecto)
        """
        return self.state_size
