#!/usr/bin/env python3
"""
Training Node para DQN Navigation

Nodo ROS2 que ejecuta el loop de entrenamiento del agente DQN.
Entrena al robot a navegar hacia goals mientras evita obstáculos.

Funcionalidades:
- Loop de entrenamiento con episodios
- Logging de métricas (recompensa, epsilon, pasos)
- Guardado automático del modelo
- Visualización opcional con matplotlib

Uso:
    ros2 run finalwork_pkg train_node
    ros2 run finalwork_pkg train_node --ros-args -p num_episodes:=500

Autor: Proyecto DQN Navigation - finalwork_pkg
"""

import os
import rclpy
from rclpy.node import Node

import numpy as np
import time
from typing import List
from datetime import datetime

from .environment_manager import EnvironmentManager
from .dqn_agent import DQNAgent


# =============================================================================
# CONFIGURACIÓN DE ENTRENAMIENTO
# =============================================================================
DEFAULT_NUM_EPISODES = 1000          # 1000 épocas de entrenamiento
DEFAULT_SAVE_INTERVAL = 100          # Guardar cada 100 episodios (10 checkpoints)
DEFAULT_TARGET_UPDATE_INTERVAL = 5   # Actualizar target cada 5 episodios
DEFAULT_MODEL_DIR = 'models'
DEFAULT_MODEL_NAME = 'dqn_model.pkl'


class TrainNode(Node):
    """
    Nodo ROS2 para entrenamiento del agente DQN.
    
    Parámetros ROS2:
    - num_episodes: Número de episodios de entrenamiento
    - save_interval: Cada cuántos episodios guardar el modelo
    - model_dir: Directorio para guardar modelos
    - model_name: Nombre del archivo del modelo
    """
    
    def __init__(self):
        super().__init__('dqn_train_node')
        
        # Declarar parámetros
        self.declare_parameter('num_episodes', DEFAULT_NUM_EPISODES)
        self.declare_parameter('save_interval', DEFAULT_SAVE_INTERVAL)
        self.declare_parameter('target_update_interval', DEFAULT_TARGET_UPDATE_INTERVAL)
        self.declare_parameter('model_dir', DEFAULT_MODEL_DIR)
        self.declare_parameter('model_name', DEFAULT_MODEL_NAME)
        
        # Obtener parámetros
        self.num_episodes = self.get_parameter('num_episodes').value
        self.save_interval = self.get_parameter('save_interval').value
        self.target_update_interval = self.get_parameter('target_update_interval').value
        self.model_dir = self.get_parameter('model_dir').value
        self.model_name = self.get_parameter('model_name').value
        
        # Crear directorio de modelos si no existe
        self._setup_model_directory()
        
        self.get_logger().info('='*60)
        self.get_logger().info('DQN Training Node Initialized')
        self.get_logger().info(f'  Episodes: {self.num_episodes}')
        self.get_logger().info(f'  Save interval: {self.save_interval}')
        self.get_logger().info(f'  Model path: {self.model_path}')
        self.get_logger().info('='*60)
        
        # Métricas de entrenamiento
        self.episode_rewards: List[float] = []
        self.episode_steps: List[int] = []
        self.goals_reached = 0
        self.collisions = 0
    
    def _setup_model_directory(self):
        """Configura el directorio para guardar modelos."""
        # Obtener directorio del paquete
        import ament_index_python
        try:
            pkg_share = ament_index_python.get_package_share_directory('finalwork_pkg')
            # Usar directorio padre (src/finalwork_pkg)
            pkg_src = os.path.dirname(os.path.dirname(pkg_share))
            if 'install' in pkg_src:
                # Buscar en src
                workspace = os.path.dirname(os.path.dirname(pkg_src))
                model_base = os.path.join(workspace, 'src', 'finalwork_pkg', self.model_dir)
            else:
                model_base = os.path.join(pkg_share, self.model_dir)
        except Exception:
            # Fallback: usar directorio actual
            model_base = os.path.join(os.getcwd(), self.model_dir)
        
        # Crear directorio si no existe
        os.makedirs(model_base, exist_ok=True)
        
        self.model_path = os.path.join(model_base, self.model_name)
        self.get_logger().info(f'Model directory: {model_base}')
    
    def run_training(self):
        """
        Ejecuta el loop de entrenamiento principal.
        """
        self.get_logger().info('Creating Environment Manager...')
        
        # Crear environment (goal aleatorio para training)
        env = EnvironmentManager()
        
        self.get_logger().info('Waiting for sensor data...')
        
        # Esperar datos iniciales
        if not env.wait_for_data(timeout_sec=10.0):
            self.get_logger().error('Failed to receive sensor data. Exiting.')
            return
        
        self.get_logger().info('Sensor data received. Creating DQN Agent...')
        
        # Crear agente DQN
        state_size = env.get_state_size()
        action_size = env.get_action_size()
        agent = DQNAgent(state_size, action_size, use_per=True)
        
        self.get_logger().info(f'Agent created: state_size={state_size}, action_size={action_size}')
        self.get_logger().info('='*60)
        self.get_logger().info('Starting Training...')
        self.get_logger().info('='*60)
        
        training_start_time = time.time()
        
        # Loop de entrenamiento
        for episode in range(1, self.num_episodes + 1):
            episode_start_time = time.time()
            
            # Reset del episodio
            state = env.reset()
            total_reward = 0.0
            steps = 0
            done = False
            
            # Loop del episodio
            while not done:
                # Seleccionar acción
                action = agent.act(state, training=True)
                
                # Ejecutar acción
                next_state, reward, done, info = env.step(action)
                
                # Almacenar experiencia
                agent.remember(state, action, reward, next_state, done)
                
                # Entrenar
                agent.replay()
                
                # Actualizar estado
                state = next_state
                total_reward += reward
                steps += 1
            
            # Actualizar target network periódicamente
            if episode % self.target_update_interval == 0:
                agent.update_target_network()
            
            # Registrar métricas
            self.episode_rewards.append(total_reward)
            self.episode_steps.append(steps)
            
            if env.goal_reached:
                self.goals_reached += 1
            if env.collision:
                self.collisions += 1
            
            # Calcular estadísticas
            episode_time = time.time() - episode_start_time
            avg_reward_last_100 = np.mean(self.episode_rewards[-100:])
            success_rate = self.goals_reached / episode * 100
            
            # Logging
            stats = agent.get_training_stats()
            self.get_logger().info(
                f'Ep {episode}/{self.num_episodes} | '
                f'Steps: {steps:3d} | '
                f'Reward: {total_reward:7.2f} | '
                f'Avg100: {avg_reward_last_100:7.2f} | '
                f'ε: {stats["epsilon"]:.3f} | '
                f'Goals: {self.goals_reached} ({success_rate:.1f}%) | '
                f'Time: {episode_time:.1f}s'
            )
            
            # Guardar modelo periódicamente
            if episode % self.save_interval == 0:
                self._save_model(agent, episode)
        
        # Entrenamiento completado
        total_time = time.time() - training_start_time
        self.get_logger().info('='*60)
        self.get_logger().info('Training Complete!')
        self.get_logger().info(f'  Total episodes: {self.num_episodes}')
        self.get_logger().info(f'  Goals reached: {self.goals_reached}')
        self.get_logger().info(f'  Collisions: {self.collisions}')
        self.get_logger().info(f'  Final success rate: {self.goals_reached/self.num_episodes*100:.1f}%')
        self.get_logger().info(f'  Total time: {total_time/60:.1f} minutes')
        self.get_logger().info('='*60)
        
        # Guardar modelo final
        self._save_model(agent, self.num_episodes, final=True)
        
        # Guardar métricas
        self._save_metrics()
        
        # Detener robot
        env.stop_robot()
        env.destroy_node()
    
    def _save_model(self, agent: DQNAgent, episode: int, final: bool = False):
        """Guarda el modelo del agente."""
        if final:
            filepath = self.model_path
        else:
            # Agregar número de episodio al nombre
            base, ext = os.path.splitext(self.model_path)
            filepath = f"{base}_ep{episode}{ext}"
        
        agent.save(filepath)
        self.get_logger().info(f'Model saved: {filepath}')
    
    def _save_metrics(self):
        """Guarda las métricas de entrenamiento."""
        import pickle
        
        metrics = {
            'episode_rewards': self.episode_rewards,
            'episode_steps': self.episode_steps,
            'goals_reached': self.goals_reached,
            'collisions': self.collisions,
            'timestamp': datetime.now().isoformat()
        }
        
        base, _ = os.path.splitext(self.model_path)
        metrics_path = f"{base}_metrics.pkl"
        
        with open(metrics_path, 'wb') as f:
            pickle.dump(metrics, f)
        
        self.get_logger().info(f'Metrics saved: {metrics_path}')


def main(args=None):
    rclpy.init(args=args)
    
    train_node = TrainNode()
    
    try:
        train_node.run_training()
    except KeyboardInterrupt:
        train_node.get_logger().info('Training interrupted by user')
    except Exception as e:
        train_node.get_logger().error(f'Training error: {e}')
        raise
    finally:
        train_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
