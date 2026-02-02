"""
Training Node Mejorado para DQN Navigation

Nodo principal que ejecuta el bucle de entrenamiento del agente DQN mejorado.

Características:
- Soporte para Double DQN y Prioritized Experience Replay
- Más episodios de entrenamiento (300+)
- Actualización más frecuente de Target Network
- Logging detallado de métricas avanzadas
- Guardado de trayectorias exitosas
- Early stopping si se alcanza tasa de éxito alta

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
# PARÁMETROS DE ENTRENAMIENTO OPTIMIZADOS
# ============================================================================
NUM_EPISODES = 1000           # Más episodios para mejor convergencia
MAX_STEPS_PER_EPISODE = 500  # Sincronizado con environment.py
SAVE_INTERVAL = 25           # Guardar más frecuentemente
TARGET_UPDATE_INTERVAL = 5   # Actualizar target más frecuentemente
LOG_INTERVAL = 1             # Log cada episodio

# Early stopping
EARLY_STOP_SUCCESS_RATE = 0.85    # Parar si alcanza 85% éxito
EARLY_STOP_WINDOW = 50            # Ventana de evaluación


def main(args=None):
    """
    Función principal para ejecutar el entrenamiento mejorado.
    """
    rclpy.init(args=args)
    
    # Crear nodo de entorno
    env_node = EnvironmentNode()
    
    # Obtener dimensiones del estado y acciones
    state_size = env_node.get_state_size()
    action_size = env_node.get_action_size()
    
    # Crear agente DQN mejorado con PER activado
    agent = DQNAgent(state_size, action_size, use_per=True)
    
    # Directorio para guardar modelos
    save_dir = os.path.expanduser('~/dqn_models')
    os.makedirs(save_dir, exist_ok=True)
    
    # Métricas de entrenamiento
    episode_rewards = []
    episode_steps = []
    episode_outcomes = []  # 'goal', 'collision', 'timeout', 'too_far'
    goals_reached = 0
    collisions = 0
    timeouts = 0
    too_far = 0
    
    # Mejores valores para guardar mejor modelo
    best_avg_reward = float('-inf')
    best_success_rate = 0.0
    
    env_node.get_logger().info('='*60)
    env_node.get_logger().info('DQN TRAINING NODE - IMPROVED VERSION')
    env_node.get_logger().info('='*60)
    env_node.get_logger().info(f'State size: {state_size}')
    env_node.get_logger().info(f'Action size: {action_size}')
    env_node.get_logger().info(f'Episodes: {NUM_EPISODES}')
    env_node.get_logger().info(f'Max steps/episode: {MAX_STEPS_PER_EPISODE}')
    env_node.get_logger().info(f'Using PER: {agent.use_per}')
    env_node.get_logger().info(f'Save directory: {save_dir}')
    env_node.get_logger().info('='*60)
    
    env_node.get_logger().info('Waiting for sensor data...')
    if not env_node.wait_for_data(timeout_sec=10.0):
        env_node.get_logger().error('Failed to receive sensor data. Exiting.')
        env_node.destroy_node()
        rclpy.shutdown()
        return
    
    env_node.get_logger().info('Sensor data received. Starting training...')
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
                action = agent.act(state, training=True)
                
                # Ejecutar acción en el entorno
                next_state, reward, done, info = env_node.step(action)
                next_state = np.reshape(next_state, [1, state_size])
                
                # Almacenar experiencia (reward clipping se hace en agent)
                agent.remember(state[0], action, reward, next_state[0], done)
                
                # Entrenar con replay
                agent.replay()
                
                # Actualizar estado
                state = next_state
                total_reward += reward
                
                if done:
                    # Registrar tipo de terminación
                    if "GOAL" in info:
                        goals_reached += 1
                        episode_outcomes.append('goal')
                        # VICTORY BOOST: Entrenar extra para reforzar el éxito inmediatamente
                        env_node.get_logger().info("🌟 SPIRIT BOOST! Repasando la lección exitosa 10 veces...")
                        for _ in range(10):
                            agent.replay()
                    elif "COLLISION" in info:
                        collisions += 1
                        episode_outcomes.append('collision')
                    elif "TIMEOUT" in info:
                        timeouts += 1
                        episode_outcomes.append('timeout')
                    elif "TOO_FAR" in info:
                        too_far += 1
                        episode_outcomes.append('too_far')
                    break
            
            episode_rewards.append(total_reward)
            episode_steps.append(step)
            
            # Actualizar target network más frecuentemente
            if episode % TARGET_UPDATE_INTERVAL == 0:
                agent.update_target_network()
            
            # Guardar modelo periódicamente
            if episode % SAVE_INTERVAL == 0:
                filepath = os.path.join(save_dir, f'model_episode_{episode}.pkl')
                agent.save(filepath)
                
                # Guardar también trayectorias
                traj_path = os.path.join(save_dir, f'trajectories_ep_{episode}.pkl')
                env_node.save_trajectories_to_file(traj_path)
            
            # Calcular métricas de ventana deslizante
            window = min(50, len(episode_rewards))
            recent_rewards = episode_rewards[-window:]
            recent_outcomes = episode_outcomes[-window:]
            
            avg_reward = np.mean(recent_rewards)
            success_rate = recent_outcomes.count('goal') / len(recent_outcomes) if recent_outcomes else 0
            
            # Guardar mejor modelo
            if avg_reward > best_avg_reward:
                best_avg_reward = avg_reward
                filepath = os.path.join(save_dir, 'best_reward_model.pkl')
                agent.save(filepath)
            
            if success_rate > best_success_rate:
                best_success_rate = success_rate
                filepath = os.path.join(save_dir, 'best_success_model.pkl')
                agent.save(filepath)
            
            # Logging de progreso
            stats = agent.get_training_stats()
            
            env_node.get_logger().info(
                f'Ep:{episode:3d}/{NUM_EPISODES} | '
                f'Steps:{step:3d} | '
                f'R:{total_reward:7.1f} | '
                f'Avg:{avg_reward:7.1f} | '
                f'SR:{100*success_rate:5.1f}% | '
                f'ε:{agent.epsilon:.3f} | '
                f'G:{goals_reached} C:{collisions} T:{timeouts}'
            )
            
            # Log detallado cada 10 episodios
            if episode % 10 == 0:
                env_node.get_logger().info(
                    f'  Stats: MemSize={stats["memory_size"]} | '
                    f'Beta={stats.get("beta", 0):.3f} | '
                    f'AvgTD={stats.get("avg_td_error", 0):.4f} | '
                    f'AvgQ={stats.get("avg_q_value", 0):.2f}'
                )
                
                # Log de trayectorias
                traj_stats = env_node.get_trajectory_stats()
                if traj_stats['total'] > 0:
                    env_node.get_logger().info(
                        f'  Trajectories: Total={traj_stats["total"]} | '
                        f'Best={traj_stats["best_steps"]} steps | '
                        f'Avg={traj_stats["avg_steps"]:.1f}'
                    )
            
            # Early stopping
            if len(episode_outcomes) >= EARLY_STOP_WINDOW:
                if success_rate >= EARLY_STOP_SUCCESS_RATE:
                    env_node.get_logger().info(
                        f'Early stopping! Success rate {100*success_rate:.1f}% >= '
                        f'{100*EARLY_STOP_SUCCESS_RATE:.1f}%'
                    )
                    break
        
        # Guardar modelo final
        filepath = os.path.join(save_dir, 'final_model.pkl')
        agent.save(filepath)
        env_node.get_logger().info(f'Final model saved: {filepath}')
        
        # Guardar trayectorias finales
        traj_path = os.path.join(save_dir, 'final_trajectories.pkl')
        env_node.save_trajectories_to_file(traj_path)
        
        # Estadísticas finales
        elapsed_time = time.time() - start_time
        final_success_rate = goals_reached / episode if episode > 0 else 0
        
        env_node.get_logger().info('='*60)
        env_node.get_logger().info('TRAINING COMPLETED')
        env_node.get_logger().info('='*60)
        env_node.get_logger().info(f'Total episodes: {episode}')
        env_node.get_logger().info(f'Total time: {elapsed_time/60:.2f} minutes')
        env_node.get_logger().info(f'Time per episode: {elapsed_time/episode:.1f} seconds')
        env_node.get_logger().info('-'*40)
        env_node.get_logger().info(f'Goals reached: {goals_reached} ({100*goals_reached/episode:.1f}%)')
        env_node.get_logger().info(f'Collisions: {collisions} ({100*collisions/episode:.1f}%)')
        env_node.get_logger().info(f'Timeouts: {timeouts} ({100*timeouts/episode:.1f}%)')
        env_node.get_logger().info(f'Too far: {too_far} ({100*too_far/episode:.1f}%)')
        env_node.get_logger().info('-'*40)
        env_node.get_logger().info(f'Avg reward: {np.mean(episode_rewards):.2f}')
        env_node.get_logger().info(f'Best avg reward (50ep): {best_avg_reward:.2f}')
        env_node.get_logger().info(f'Best success rate: {100*best_success_rate:.1f}%')
        env_node.get_logger().info(f'Avg steps: {np.mean(episode_steps):.1f}')
        env_node.get_logger().info(f'Final epsilon: {agent.epsilon:.4f}')
        
        traj_stats = env_node.get_trajectory_stats()
        if traj_stats['total'] > 0:
            env_node.get_logger().info('-'*40)
            env_node.get_logger().info(f'Successful trajectories: {traj_stats["total"]}')
            env_node.get_logger().info(f'Best trajectory: {traj_stats["best_steps"]} steps')
        
        env_node.get_logger().info('='*60)
        
    except KeyboardInterrupt:
        env_node.get_logger().info('Training interrupted by user')
        # Guardar modelo de emergencia
        filepath = os.path.join(save_dir, 'interrupted_model.pkl')
        agent.save(filepath)
        env_node.get_logger().info(f'Interrupted model saved: {filepath}')
        
    finally:
        # Cleanup
        env_node.stop_robot()
        env_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
