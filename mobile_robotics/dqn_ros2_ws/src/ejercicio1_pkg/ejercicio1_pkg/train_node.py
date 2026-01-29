"""
Training Node para DQN Navigation

Nodo principal que ejecuta el bucle de entrenamiento del agente DQN.

Características:
- 200+ episodios de entrenamiento
- Máximo 500 pasos por episodio
- Guardado periódico del modelo (cada 50 episodios)
- Actualización periódica de la Target Network (cada 10 episodios)
- Logging detallado de métricas

Autor: Proyecto Académico DQN Navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

import numpy as np
import os
import time
from datetime import datetime

from .dqn_agent import DQNAgent, EPSILON_MIN
from .environment import EnvironmentNode


# ============================================================================
# PARÁMETROS DE ENTRENAMIENTO
# ============================================================================
NUM_EPISODES = 200           # Número de episodios de entrenamiento
MAX_STEPS_PER_EPISODE = 500  # Máximo de pasos por episodio
SAVE_INTERVAL = 50           # Guardar modelo cada N episodios
TARGET_UPDATE_INTERVAL = 10  # Actualizar target network cada N episodios
STEP_FREQUENCY = 10.0        # Frecuencia de pasos (Hz)


class TrainingNode(Node):
    """
    Nodo ROS2 para entrenamiento del agente DQN.
    """
    
    def __init__(self, env: EnvironmentNode):
        super().__init__('dqn_training')
        
        self.env = env
        
        # Obtener dimensiones del estado y acciones
        self.state_size = env.get_state_size()
        self.action_size = env.get_action_size()
        
        # Crear agente DQN
        self.agent = DQNAgent(self.state_size, self.action_size)
        
        # Directorio para guardar modelos
        self.save_dir = os.path.expanduser('~/dqn_models')
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Métricas de entrenamiento
        self.episode_rewards = []
        self.episode_steps = []
        self.goals_reached = 0
        self.collisions = 0
        
        self.get_logger().info('='*60)
        self.get_logger().info('DQN Training Node Initialized')
        self.get_logger().info(f'State size: {self.state_size}')
        self.get_logger().info(f'Action size: {self.action_size}')
        self.get_logger().info(f'Episodes: {NUM_EPISODES}')
        self.get_logger().info(f'Max steps/episode: {MAX_STEPS_PER_EPISODE}')
        self.get_logger().info(f'Save directory: {self.save_dir}')
        self.get_logger().info('='*60)
    
    def train(self):
        """
        Bucle principal de entrenamiento.
        """
        self.get_logger().info('Starting training...')
        
        # Esperar datos iniciales
        self.get_logger().info('Waiting for sensor data...')
        if not self.env.wait_for_data(timeout_sec=10.0):
            self.get_logger().error('Failed to receive sensor data. Exiting.')
            return
        
        self.get_logger().info('Sensor data received. Starting episodes...')
        
        start_time = time.time()
        
        for episode in range(1, NUM_EPISODES + 1):
            episode_reward, episode_steps = self._run_episode(episode)
            
            self.episode_rewards.append(episode_reward)
            self.episode_steps.append(episode_steps)
            
            # Actualizar target network periódicamente
            if episode % TARGET_UPDATE_INTERVAL == 0:
                self.agent.update_target_network()
                self.get_logger().info(f'Target network updated at episode {episode}')
            
            # Guardar modelo periódicamente
            if episode % SAVE_INTERVAL == 0:
                self._save_model(episode)
            
            # Logging de progreso
            self._log_progress(episode, episode_reward, episode_steps)
        
        # Guardar modelo final
        self._save_model(NUM_EPISODES, final=True)
        
        # Estadísticas finales
        elapsed_time = time.time() - start_time
        self._log_final_stats(elapsed_time)
    
    def _run_episode(self, episode_num: int) -> tuple:
        """
        Ejecuta un episodio completo.
        
        Args:
            episode_num: Número del episodio actual
            
        Returns:
            Tuple de (recompensa_total, número_de_pasos)
        """
        # Reset del entorno
        state = self.env.reset()
        state = np.reshape(state, [1, self.state_size])
        
        total_reward = 0.0
        step = 0
        
        rate = self.create_rate(STEP_FREQUENCY)
        
        for step in range(1, MAX_STEPS_PER_EPISODE + 1):
            # Seleccionar acción (epsilon-greedy)
            action = self.agent.act(state)
            
            # Ejecutar acción en el entorno
            next_state, reward, done, info = self.env.step(action)
            next_state = np.reshape(next_state, [1, self.state_size])
            
            # Almacenar experiencia en el buffer
            self.agent.remember(state[0], action, reward, next_state[0], done)
            
            # Entrenar con replay
            self.agent.replay()
            
            # Actualizar estado
            state = next_state
            total_reward += reward
            
            if done:
                # Registrar tipo de terminación
                if "GOAL" in info:
                    self.goals_reached += 1
                elif "COLLISION" in info:
                    self.collisions += 1
                break
            
            # Mantener frecuencia de control
            rate.sleep()
        
        return total_reward, step
    
    def _save_model(self, episode: int, final: bool = False):
        """
        Guarda el modelo actual.
        
        Args:
            episode: Número del episodio
            final: True si es el modelo final
        """
        if final:
            filename = 'trained_model.pkl'
        else:
            filename = f'model_episode_{episode}.pkl'
        
        filepath = os.path.join(self.save_dir, filename)
        self.agent.save(filepath)
        self.get_logger().info(f'Model saved: {filepath}')
    
    def _log_progress(self, episode: int, reward: float, steps: int):
        """
        Registra el progreso del entrenamiento.
        """
        # Calcular promedio de las últimas 10 recompensas
        recent_rewards = self.episode_rewards[-10:]
        avg_reward = np.mean(recent_rewards) if recent_rewards else 0.0
        
        self.get_logger().info(
            f'Episode: {episode}/{NUM_EPISODES} | '
            f'Steps: {steps} | '
            f'Reward: {reward:.2f} | '
            f'Avg(10): {avg_reward:.2f} | '
            f'Epsilon: {self.agent.epsilon:.3f} | '
            f'Goals: {self.goals_reached} | '
            f'Collisions: {self.collisions}'
        )
    
    def _log_final_stats(self, elapsed_time: float):
        """
        Registra estadísticas finales del entrenamiento.
        """
        self.get_logger().info('='*60)
        self.get_logger().info('TRAINING COMPLETED')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Total episodes: {NUM_EPISODES}')
        self.get_logger().info(f'Total time: {elapsed_time/60:.2f} minutes')
        self.get_logger().info(f'Goals reached: {self.goals_reached}')
        self.get_logger().info(f'Collisions: {self.collisions}')
        self.get_logger().info(f'Success rate: {100*self.goals_reached/NUM_EPISODES:.1f}%')
        self.get_logger().info(f'Avg reward: {np.mean(self.episode_rewards):.2f}')
        self.get_logger().info(f'Avg steps: {np.mean(self.episode_steps):.1f}')
        self.get_logger().info(f'Final epsilon: {self.agent.epsilon:.4f}')
        self.get_logger().info('='*60)


def main(args=None):
    """
    Función principal para ejecutar el entrenamiento.
    """
    rclpy.init(args=args)
    
    # Crear nodo de entorno
    env_node = EnvironmentNode()
    
    # Crear nodo de entrenamiento
    train_node = TrainingNode(env_node)
    
    try:
        # Ejecutar entrenamiento
        train_node.train()
    except KeyboardInterrupt:
        train_node.get_logger().info('Training interrupted by user')
    finally:
        # Cleanup
        env_node.stop_robot()
        env_node.destroy_node()
        train_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
