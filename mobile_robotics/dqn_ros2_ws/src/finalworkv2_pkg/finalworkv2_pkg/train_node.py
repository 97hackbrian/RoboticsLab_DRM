"""
DQN Training/Testing Node for ROS2.

Main entry point for training the DQN agent or testing with a trained model.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import numpy as np

from .dqn_agent import DQNAgent
from .replay_buffer import ReplayBuffer
from .environment_manager import EnvironmentManager


class TrainNode(Node):
    """
    ROS2 Node for DQN navigation training and testing.
    
    Parameters:
        mode (str): 'train' or 'test'
        goal_x (float): Goal X position (test mode)
        goal_y (float): Goal Y position (test mode)
        max_episodes (int): Maximum training episodes
        max_steps_per_episode (int): Maximum steps per episode
        batch_size (int): Training batch size
        gamma (float): Discount factor
        epsilon_start (float): Initial exploration rate
        epsilon_end (float): Minimum exploration rate
        epsilon_decay (float): Exploration decay rate
        learning_rate (float): Neural network learning rate
        hidden_layers (list): Hidden layer sizes
        buffer_size (int): Replay buffer capacity
        target_update_freq (int): Target network update frequency
        goal_radius (float): Goal reached threshold
        collision_distance (float): Collision detection threshold
        max_linear_vel (float): Maximum linear velocity
        max_angular_vel (float): Maximum angular velocity
        model_path (str): Path to save/load model
        save_interval (int): Episodes between saves
    """
    
    def __init__(self):
        super().__init__('train_node')
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self.mode = self.get_parameter('mode').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.max_episodes = self.get_parameter('max_episodes').value
        self.max_steps = self.get_parameter('max_steps_per_episode').value
        self.batch_size = self.get_parameter('batch_size').value
        self.gamma = self.get_parameter('gamma').value
        self.epsilon_start = self.get_parameter('epsilon_start').value
        self.epsilon_end = self.get_parameter('epsilon_end').value
        self.epsilon_decay = self.get_parameter('epsilon_decay').value
        self.learning_rate = self.get_parameter('learning_rate').value
        hidden_layers_list = self.get_parameter('hidden_layers').value
        self.hidden_layers = tuple(hidden_layers_list) if hidden_layers_list else (128, 64)
        self.buffer_size = self.get_parameter('buffer_size').value
        self.target_update_freq = self.get_parameter('target_update_freq').value
        self.goal_radius = self.get_parameter('goal_radius').value
        self.collision_distance = self.get_parameter('collision_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.model_path = self.get_parameter('model_path').value
        self.save_interval = self.get_parameter('save_interval').value
        
        self.get_logger().info(f"Mode: {self.mode}")
        self.get_logger().info(f"Hidden layers: {self.hidden_layers}")
        
        # Initialize environment
        self.env = EnvironmentManager(
            node=self,
            goal_radius=self.goal_radius,
            collision_distance=self.collision_distance,
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel
        )
        
        # Initialize DQN agent
        self.agent = DQNAgent(
            state_size=self.env.state_size,
            action_size=self.env.action_size,
            hidden_layers=self.hidden_layers,
            learning_rate=self.learning_rate,
            gamma=self.gamma,
            epsilon_start=self.epsilon_start,
            epsilon_end=self.epsilon_end,
            epsilon_decay=self.epsilon_decay,
            target_update_freq=self.target_update_freq
        )
        
        # Initialize replay buffer
        self.buffer = ReplayBuffer(capacity=self.buffer_size)
        
        # Try to load existing model
        if os.path.exists(self.model_path):
            if self.agent.load(self.model_path):
                self.get_logger().info(f"Loaded model from {self.model_path}")
            else:
                self.get_logger().warn(f"Failed to load model from {self.model_path}")
        
        # Statistics
        self.episode_rewards = []
        self.episode_steps = []
        self.success_count = 0
        
        self.get_logger().info("TrainNode initialized successfully")
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters with defaults."""
        self.declare_parameter('mode', 'train')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('max_episodes', 500)
        self.declare_parameter('max_steps_per_episode', 500)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('gamma', 0.99)
        self.declare_parameter('epsilon_start', 1.0)
        self.declare_parameter('epsilon_end', 0.01)
        self.declare_parameter('epsilon_decay', 0.995)
        self.declare_parameter('learning_rate', 0.001)
        self.declare_parameter('hidden_layers', [128, 64])
        self.declare_parameter('buffer_size', 10000)
        self.declare_parameter('target_update_freq', 100)
        self.declare_parameter('goal_radius', 0.3)
        self.declare_parameter('collision_distance', 0.25)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('model_path', 'models/dqn_model.pkl')
        self.declare_parameter('save_interval', 50)
    
    def run(self):
        """Main entry point - run training or testing."""
        if self.mode == 'train':
            self._run_training()
        elif self.mode == 'test':
            self._run_testing()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
    
    def _run_training(self):
        """Run the training loop."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Starting DQN Training")
        self.get_logger().info("=" * 50)
        
        for episode in range(self.max_episodes):
            # Reset environment (random goal)
            state = self.env.reset()
            episode_reward = 0.0
            step = 0
            
            while step < self.max_steps:
                # Select action
                action = self.agent.select_action(state, training=True)
                
                # OBSTACLE-REACTIVE OVERRIDE: If too close to obstacle, force safe action
                # Check laser readings in state (first 20 elements, normalized)
                min_laser_normalized = np.min(state[:20])
                if min_laser_normalized < 0.05:  # Very close (< 0.5m with 10m max range)
                    # Force backup or rotation
                    action = 5  # Backup
                elif min_laser_normalized < 0.08:  # Close
                    # Prefer rotation over forward
                    if action == 0:  # Was going to go forward
                        action = np.random.choice([3, 4])  # Rotate instead
                
                # Execute action
                next_state, reward, done = self.env.step(action)
                
                # Store experience
                self.buffer.store(state, action, reward, next_state, done)
                
                # Train if buffer has enough samples
                if self.buffer.is_ready(self.batch_size):
                    batch = self.buffer.sample(self.batch_size)
                    loss = self.agent.train(*batch)
                
                episode_reward += reward
                state = next_state
                step += 1
                
                if done:
                    if reward > 50:  # Goal reached
                        self.success_count += 1
                    break
            
            # Decay epsilon
            self.agent.decay_epsilon()
            
            # Log statistics
            self.episode_rewards.append(episode_reward)
            self.episode_steps.append(step)
            
            avg_reward = np.mean(self.episode_rewards[-100:])
            success_rate = self.success_count / (episode + 1) * 100
            
            self.get_logger().info(
                f"Episode {episode+1}/{self.max_episodes} | "
                f"Steps: {step} | "
                f"Reward: {episode_reward:.2f} | "
                f"Avg(100): {avg_reward:.2f} | "
                f"Epsilon: {self.agent.epsilon:.3f} | "
                f"Success: {success_rate:.1f}%"
            )
            
            # Save model periodically
            if (episode + 1) % self.save_interval == 0:
                self.agent.save(self.model_path)
                self.get_logger().info(f"Model saved to {self.model_path}")
        
        # Final save
        self.agent.save(self.model_path)
        self.get_logger().info("Training complete!")
        self.get_logger().info(f"Final success rate: {self.success_count}/{self.max_episodes}")
        
        # Stop robot
        self.env.stop()
    
    def _run_testing(self):
        """Run testing with a trained model."""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Starting DQN Testing")
        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y})")
        self.get_logger().info("=" * 50)
        
        # Disable exploration
        self.agent.epsilon = 0.0
        
        # Reset with specified goal
        state = self.env.reset(goal_x=self.goal_x, goal_y=self.goal_y)
        total_reward = 0.0
        step = 0
        
        while step < self.max_steps:
            # Select best action (no exploration)
            action = self.agent.select_action(state, training=False)
            
            # Log decision info
            q_values = self.agent.get_q_values(state)
            robot_x, robot_y, robot_yaw = self.env.get_robot_pose()
            goal_x, goal_y, dist = self.env.get_goal_info()
            
            if step % 20 == 0:
                self.get_logger().info(
                    f"Step {step} | "
                    f"Pos: ({robot_x:.2f}, {robot_y:.2f}) | "
                    f"Dist: {dist:.2f}m | "
                    f"Action: {action} | "
                    f"Q-values: {q_values}"
                )
            
            # SAFETY OVERRIDE: Force safe actions when too close to obstacles
            min_laser_normalized = np.min(state[:20])
            if min_laser_normalized < 0.05:  # Very close
                action = 5  # Backup
            elif min_laser_normalized < 0.08:
                if action == 0:  # Was going forward
                    action = np.random.choice([3, 4])  # Rotate instead
            
            # Execute action
            next_state, reward, done = self.env.step(action)
            total_reward += reward
            state = next_state
            step += 1
            
            if done:
                if reward > 50:
                    self.get_logger().info("SUCCESS! Goal reached!")
                else:
                    self.get_logger().info("Episode ended (collision or timeout)")
                break
        
        self.get_logger().info(f"Testing complete. Total reward: {total_reward:.2f}")
        
        # Stop robot
        self.env.stop()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = TrainNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.env.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
