"""
DQN Agent using sklearn MLPRegressor for Q-value approximation.

This implementation uses scikit-learn's MLPRegressor as requested,
with a dual-network architecture (online + target networks).
"""

import numpy as np
import pickle
import os
from typing import Tuple, Optional, List
from sklearn.neural_network import MLPRegressor
from sklearn.exceptions import NotFittedError
import warnings

# Suppress sklearn convergence warnings during incremental training
warnings.filterwarnings('ignore', category=UserWarning)


class DQNAgent:
    """
    Deep Q-Network agent using sklearn MLPRegressor.
    
    Features:
    - Epsilon-greedy exploration
    - Experience replay training
    - Target network for stability (cloned periodically)
    - Model save/load functionality
    """
    
    def __init__(
        self,
        state_size: int,
        action_size: int,
        hidden_layers: Tuple[int, ...] = (128, 64),
        learning_rate: float = 0.001,
        gamma: float = 0.99,
        epsilon_start: float = 1.0,
        epsilon_end: float = 0.01,
        epsilon_decay: float = 0.995,
        target_update_freq: int = 100
    ):
        """
        Initialize the DQN agent.
        
        Args:
            state_size: Dimension of state space
            action_size: Number of discrete actions
            hidden_layers: Tuple of hidden layer sizes
            learning_rate: Learning rate for the neural network
            gamma: Discount factor for future rewards
            epsilon_start: Initial exploration rate
            epsilon_end: Minimum exploration rate
            epsilon_decay: Decay rate per episode
            target_update_freq: Steps between target network updates
        """
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.target_update_freq = target_update_freq
        self.train_step_counter = 0
        
        # Online Q-network
        self.q_network = MLPRegressor(
            hidden_layer_sizes=hidden_layers,
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,  # Allow incremental training
            random_state=42
        )
        
        # Target Q-network (initialized identically)
        self.target_network = MLPRegressor(
            hidden_layer_sizes=hidden_layers,
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )
        
        # Flag to track if networks have been fitted
        self._is_fitted = False
        
        # Initialize networks with dummy data
        self._initialize_networks()
    
    def _initialize_networks(self) -> None:
        """Initialize networks with dummy data to set up internal structure."""
        dummy_state = np.zeros((1, self.state_size))
        dummy_q_values = np.zeros((1, self.action_size))
        
        # Fit with dummy data to initialize
        self.q_network.fit(dummy_state, dummy_q_values)
        self.target_network.fit(dummy_state, dummy_q_values)
        self._is_fitted = True
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """
        Select action using epsilon-greedy policy.
        
        Args:
            state: Current state observation
            training: If True, use epsilon-greedy; if False, use greedy
            
        Returns:
            Selected action index
        """
        if training and np.random.random() < self.epsilon:
            return np.random.randint(self.action_size)
        
        # Get Q-values for all actions
        state = state.reshape(1, -1)
        q_values = self.q_network.predict(state)
        return int(np.argmax(q_values[0]))
    
    def train(self, states: np.ndarray, actions: np.ndarray, rewards: np.ndarray,
              next_states: np.ndarray, dones: np.ndarray) -> float:
        """
        Train the Q-network on a batch of experiences.
        
        Uses the Bellman equation:
        Q(s,a) = r + γ * max_a' Q_target(s', a') * (1 - done)
        
        Args:
            states: Batch of states
            actions: Batch of actions taken
            rewards: Batch of rewards received
            next_states: Batch of next states
            dones: Batch of done flags
            
        Returns:
            Training loss (MSE)
        """
        batch_size = len(states)
        
        # Get current Q-values
        current_q = self.q_network.predict(states)
        
        # Get next Q-values from target network
        next_q = self.target_network.predict(next_states)
        max_next_q = np.max(next_q, axis=1)
        
        # Compute target Q-values using Bellman equation
        target_q = current_q.copy()
        for i in range(batch_size):
            if dones[i]:
                target_q[i, actions[i]] = rewards[i]
            else:
                target_q[i, actions[i]] = rewards[i] + self.gamma * max_next_q[i]
        
        # Train the Q-network
        self.q_network.fit(states, target_q)
        
        # Compute loss for logging
        new_q = self.q_network.predict(states)
        loss = np.mean((target_q - new_q) ** 2)
        
        # Update target network periodically
        self.train_step_counter += 1
        if self.train_step_counter % self.target_update_freq == 0:
            self._update_target_network()
        
        return loss
    
    def _update_target_network(self) -> None:
        """Copy weights from online network to target network."""
        # For sklearn, we need to copy the internal coefficients
        self.target_network.coefs_ = [c.copy() for c in self.q_network.coefs_]
        self.target_network.intercepts_ = [i.copy() for i in self.q_network.intercepts_]
    
    def decay_epsilon(self) -> None:
        """Decay exploration rate after each episode."""
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)
    
    def save(self, filepath: str) -> None:
        """
        Save the agent to disk.
        
        Args:
            filepath: Path to save the model
        """
        os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.', exist_ok=True)
        
        save_data = {
            'q_network': self.q_network,
            'target_network': self.target_network,
            'state_size': self.state_size,
            'action_size': self.action_size,
            'epsilon': self.epsilon,
            'gamma': self.gamma,
            'train_step_counter': self.train_step_counter
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(save_data, f)
    
    def load(self, filepath: str) -> bool:
        """
        Load the agent from disk.
        
        Args:
            filepath: Path to load the model from
            
        Returns:
            True if loaded successfully, False otherwise
        """
        if not os.path.exists(filepath):
            return False
        
        try:
            with open(filepath, 'rb') as f:
                save_data = pickle.load(f)
            
            self.q_network = save_data['q_network']
            self.target_network = save_data['target_network']
            self.state_size = save_data['state_size']
            self.action_size = save_data['action_size']
            self.epsilon = save_data['epsilon']
            self.gamma = save_data['gamma']
            self.train_step_counter = save_data['train_step_counter']
            self._is_fitted = True
            
            return True
        except Exception as e:
            print(f"Error loading model: {e}")
            return False
    
    def get_q_values(self, state: np.ndarray) -> np.ndarray:
        """
        Get Q-values for all actions given a state.
        
        Args:
            state: State observation
            
        Returns:
            Array of Q-values for each action
        """
        state = state.reshape(1, -1)
        return self.q_network.predict(state)[0]
