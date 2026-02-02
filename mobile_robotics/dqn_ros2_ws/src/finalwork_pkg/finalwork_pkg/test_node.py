#!/usr/bin/env python3
"""
Test Node para DQN Navigation

Nodo ROS2 que ejecuta el agente DQN entrenado en modo inferencia.
Permite especificar un goal manual y observar el comportamiento del robot.

Funcionalidades:
- Carga modelo entrenado
- Goal manual especificado por parámetros
- Modo explotación (epsilon mínimo)
- Logging del recorrido

Uso:
    ros2 run finalwork_pkg test_node --ros-args \\
        -p model_path:=/path/to/model.pkl \\
        -p goal_x:=5.0 \\
        -p goal_y:=5.0

Autor: Proyecto DQN Navigation - finalwork_pkg
"""

import os
import rclpy
from rclpy.node import Node

import numpy as np
import time
from typing import Optional

from .environment_manager import EnvironmentManager
from .dqn_agent import DQNAgent


# =============================================================================
# CONFIGURACIÓN DE TEST
# =============================================================================
DEFAULT_NUM_EPISODES = 100
DEFAULT_MAX_STEPS = 500


class TestNode(Node):
    """
    Nodo ROS2 para testing/inferencia del agente DQN entrenado.
    
    Parámetros ROS2:
    - model_path: Ruta al modelo entrenado (requerido)
    - goal_x: Coordenada X del objetivo (requerido)
    - goal_y: Coordenada Y del objetivo (requerido)
    - num_episodes: Número de episodios de prueba
    - max_steps: Pasos máximos por episodio
    """
    
    def __init__(self):
        super().__init__('dqn_test_node')
        
        # Declarar parámetros
        self.declare_parameter('model_path', '')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('num_episodes', DEFAULT_NUM_EPISODES)
        self.declare_parameter('max_steps', DEFAULT_MAX_STEPS)
        
        # Obtener parámetros
        self.model_path = self.get_parameter('model_path').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.num_episodes = self.get_parameter('num_episodes').value
        self.max_steps = self.get_parameter('max_steps').value
        
        # Validar parámetros
        if not self.model_path:
            self.get_logger().error('model_path parameter is required!')
            raise ValueError('model_path parameter is required')
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Model file not found: {self.model_path}')
            raise FileNotFoundError(f'Model file not found: {self.model_path}')
        
        self.get_logger().info('='*60)
        self.get_logger().info('DQN Test Node Initialized')
        self.get_logger().info(f'  Model: {self.model_path}')
        self.get_logger().info(f'  Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')
        self.get_logger().info(f'  Episodes: {self.num_episodes}')
        self.get_logger().info(f'  Max steps: {self.max_steps}')
        self.get_logger().info('='*60)
        
        # Métricas de test
        self.goals_reached = 0
        self.collisions = 0
        self.timeouts = 0
        self.total_steps = 0
    
    def run_test(self):
        """
        Ejecuta el loop de testing.
        """
        self.get_logger().info('Creating Environment Manager with fixed goal...')
        
        # Crear environment con goal fijo
        env = EnvironmentManager(goal_x=self.goal_x, goal_y=self.goal_y)
        
        self.get_logger().info('Waiting for sensor data...')
        
        # Esperar datos iniciales
        if not env.wait_for_data(timeout_sec=10.0):
            self.get_logger().error('Failed to receive sensor data. Exiting.')
            return
        
        self.get_logger().info('Sensor data received. Loading DQN Agent...')
        
        # Crear y cargar agente
        state_size = env.get_state_size()
        action_size = env.get_action_size()
        agent = DQNAgent(state_size, action_size, use_per=False)
        agent.load(self.model_path)
        
        self.get_logger().info(f'Agent loaded: state_size={state_size}, action_size={action_size}')
        self.get_logger().info(f'Agent epsilon: {agent.epsilon:.4f}')
        self.get_logger().info('='*60)
        self.get_logger().info('Starting Test...')
        self.get_logger().info('='*60)
        
        # Loop de testing
        for episode in range(1, self.num_episodes + 1):
            episode_start_time = time.time()
            
            # Reset del episodio
            state = env.reset()
            total_reward = 0.0
            steps = 0
            done = False
            
            self.get_logger().info(f'\n--- Episode {episode} ---')
            self.get_logger().info(f'Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})')
            
            # Loop del episodio
            while not done and steps < self.max_steps:
                # Seleccionar acción (modo testing = epsilon mínimo)
                action = agent.act(state, training=False)
                
                # Ejecutar acción
                next_state, reward, done, info = env.step(action)
                
                # Actualizar estado
                state = next_state
                total_reward += reward
                steps += 1
                
                # Logging periódico
                if steps % 50 == 0:
                    self.get_logger().info(
                        f'  Step {steps}: dist={info["distance_to_goal"]:.2f}m, '
                        f'min_obs={info["min_obstacle_dist"]:.2f}m'
                    )
            
            # Registrar resultado
            episode_time = time.time() - episode_start_time
            self.total_steps += steps
            
            if env.goal_reached:
                self.goals_reached += 1
                result = '✓ GOAL REACHED'
            elif env.collision:
                self.collisions += 1
                result = '✗ COLLISION'
            else:
                self.timeouts += 1
                result = '○ TIMEOUT'
            
            self.get_logger().info(
                f'Episode {episode} Complete: {result} | '
                f'Steps: {steps} | '
                f'Reward: {total_reward:.2f} | '
                f'Time: {episode_time:.1f}s'
            )
        
        # Resumen final
        success_rate = self.goals_reached / self.num_episodes * 100
        avg_steps = self.total_steps / self.num_episodes
        
        self.get_logger().info('='*60)
        self.get_logger().info('Test Complete!')
        self.get_logger().info(f'  Total episodes: {self.num_episodes}')
        self.get_logger().info(f'  Goals reached: {self.goals_reached} ({success_rate:.1f}%)')
        self.get_logger().info(f'  Collisions: {self.collisions}')
        self.get_logger().info(f'  Timeouts: {self.timeouts}')
        self.get_logger().info(f'  Average steps: {avg_steps:.1f}')
        self.get_logger().info('='*60)
        
        # Detener robot
        env.stop_robot()
        env.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = TestNode()
        test_node.run_test()
    except KeyboardInterrupt:
        pass
    except ValueError as e:
        # Parámetros faltantes
        print(f'Error: {e}')
    except FileNotFoundError as e:
        # Modelo no encontrado
        print(f'Error: {e}')
    except Exception as e:
        print(f'Test error: {e}')
        raise
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
