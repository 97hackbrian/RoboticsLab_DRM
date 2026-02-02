"""
finalwork_pkg - DQN Navigation System for ROS2

Sistema de navegación basado en Deep Q-Network (DQN) para robots móviles
usando Stage ROS2 como simulador.

Módulos:
- state_processor: Procesa datos LIDAR y objetivo en vector de estado
- environment_manager: Wrapper del entorno Stage con interfaz Gym
- dqn_agent: Agente DQN con sklearn MLPRegressor
- train_node: Nodo de entrenamiento
- test_node: Nodo de testing/inferencia
"""

__version__ = '1.0.0'
__author__ = 'hackbrian'
