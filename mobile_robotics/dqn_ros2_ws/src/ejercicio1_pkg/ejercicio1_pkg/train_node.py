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


def main(args=None):
    """
    Función principal para ejecutar el entrenamiento.
    """
    rclpy.init(args=args)
    
    # Crear nodo de entorno
    env_node = EnvironmentNode()
    
    # Obtener dimensiones del estado y acciones
    state_size = env_node.get_state_size()
    action_size = env_node.get_action_size()
    
    # Crear agente DQN
    agent = DQNAgent(state_size, action_size)
    
    # Directorio para guardar modelos
    save_dir = os.path.expanduser('~/dqn_models')
    os.makedirs(save_dir, exist_ok=True)
    
    # Métricas de entrenamiento
    episode_rewards = []
    episode_steps = []
    goals_reached = 0
    collisions = 0
    
    env_node.get_logger().info('='*60)
    env_node.get_logger().info('DQN Training Node Initialized')
    env_node.get_logger().info(f'State size: {state_size}')
    env_node.get_logger().info(f'Action size: {action_size}')
    env_node.get_logger().info(f'Episodes: {NUM_EPISODES}')
    env_node.get_logger().info(f'Max steps/episode: {MAX_STEPS_PER_EPISODE}')
    env_node.get_logger().info(f'Save directory: {save_dir}')
    env_node.get_logger().info('='*60)
    
    env_node.get_logger().info('Starting training...')
    
    # Esperar datos iniciales
    env_node.get_logger().info('Waiting for sensor data...')
    if not env_node.wait_for_data(timeout_sec=10.0):
        env_node.get_logger().error('Failed to receive sensor data. Exiting.')
        env_node.destroy_node()
        rclpy.shutdown()
        return
    
    env_node.get_logger().info('Sensor data received. Starting episodes...')
    
    start_time = time.time()
    
    try:
        for episode in range(1, NUM_EPISODES + 1):
            # Reset del entorno
            state = env_node.reset()
            state = np.reshape(state, [1, state_size])
            
            total_reward = 0.0
            step = 0
            
            for step in range(1, MAX_STEPS_PER_EPISODE + 1):
                # Seleccionar acción (epsilon-greedy)
                action = agent.act(state)
                
                # Ejecutar acción en el entorno
                # El step() ya maneja el timing internamente
                next_state, reward, done, info = env_node.step(action)
                next_state = np.reshape(next_state, [1, state_size])
                
                # Almacenar experiencia en el buffer
                agent.remember(state[0], action, reward, next_state[0], done)
                
                # Entrenar con replay (esto es rápido, no bloquea mucho)
                agent.replay()
                
                # Actualizar estado
                state = next_state
                total_reward += reward
                
                if done:
                    # Registrar tipo de terminación
                    if "GOAL" in info:
                        goals_reached += 1
                    elif "COLLISION" in info:
                        collisions += 1
                    break
            
            episode_rewards.append(total_reward)
            episode_steps.append(step)
            
            # Actualizar target network periódicamente
            if episode % TARGET_UPDATE_INTERVAL == 0:
                agent.update_target_network()
                env_node.get_logger().info(f'Target network updated at episode {episode}')
            
            # Guardar modelo periódicamente
            if episode % SAVE_INTERVAL == 0:
                filepath = os.path.join(save_dir, f'model_episode_{episode}.pkl')
                agent.save(filepath)
                env_node.get_logger().info(f'Model saved: {filepath}')
            
            # Logging de progreso
            recent_rewards = episode_rewards[-10:]
            avg_reward = np.mean(recent_rewards) if recent_rewards else 0.0
            
            env_node.get_logger().info(
                f'Ep: {episode}/{NUM_EPISODES} | '
                f'Steps: {step} | '
                f'Reward: {total_reward:.1f} | '
                f'Avg(10): {avg_reward:.1f} | '
                f'Eps: {agent.epsilon:.3f} | '
                f'Goals: {goals_reached} | '
                f'Collisions: {collisions}'
            )
        
        # Guardar modelo final
        filepath = os.path.join(save_dir, 'trained_model.pkl')
        agent.save(filepath)
        env_node.get_logger().info(f'Final model saved: {filepath}')
        
        # Estadísticas finales
        elapsed_time = time.time() - start_time
        env_node.get_logger().info('='*60)
        env_node.get_logger().info('TRAINING COMPLETED')
        env_node.get_logger().info('='*60)
        env_node.get_logger().info(f'Total episodes: {NUM_EPISODES}')
        env_node.get_logger().info(f'Total time: {elapsed_time/60:.2f} minutes')
        env_node.get_logger().info(f'Goals reached: {goals_reached}')
        env_node.get_logger().info(f'Collisions: {collisions}')
        env_node.get_logger().info(f'Success rate: {100*goals_reached/NUM_EPISODES:.1f}%')
        env_node.get_logger().info(f'Avg reward: {np.mean(episode_rewards):.2f}')
        env_node.get_logger().info(f'Avg steps: {np.mean(episode_steps):.1f}')
        env_node.get_logger().info(f'Final epsilon: {agent.epsilon:.4f}')
        env_node.get_logger().info('='*60)
        
    except KeyboardInterrupt:
        env_node.get_logger().info('Training interrupted by user')
    finally:
        # Cleanup
        env_node.stop_robot()
        env_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
