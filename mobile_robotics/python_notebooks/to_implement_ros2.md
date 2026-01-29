# Deep Q-Network (DQN) for Robot Navigation - Guided Lab

## Lab Overview

**Duration**: 4-6 hours  
**Difficulty**: Intermediate  
**Prerequisites**: Basic Python, ROS2 fundamentals, understanding of reinforcement learning concepts

### Learning Objectives

By the end of this lab, you will:
1. Understand the DQN algorithm and its application to robot navigation
2. Process 2D LiDAR data for state representation
3. Design reward functions for navigation tasks
4. Implement a DQN agent using scikit-learn's MLPRegressor
5. Train and evaluate a robot navigation policy in Gazebo simulation
6. Analyze training performance and debug common issues

---
# Useful tools 

- Reset Gazebo environment.
```sh
ros2 service call /reset_world std_srvs/srv/Empty
```
- Log odom position.
```sh
ros2 topic echo /odom | grep -A 3 "position:"
```
---
## Table of Contents

1. [Introduction](#1-introduction)
2. [Theoretical Background](#2-theoretical-background)
3. [Environment Setup](#3-environment-setup)
4. [System Architecture](#4-system-architecture)
5. [Implementation](#5-implementation)
6. [Training Process](#6-training-process)
7. [Evaluation and Testing](#7-evaluation-and-testing)
8. [Troubleshooting](#8-troubleshooting)
9. [Extensions and Challenges](#9-extensions-and-challenges)

---

## 1. Introduction

### 1.1 Problem Statement

You will train a TurtleBot3 robot to navigate to a goal position while avoiding obstacles using only 2D LiDAR sensor data. The robot must learn this behavior through trial and error using Deep Q-Learning.

### 1.2 Navigation Task

- **Input**: 2D LiDAR scans (360 distance measurements)
- **Output**: Discrete actions (forward, turn left, turn right, etc.)
- **Goal**: Reach target location while avoiding collisions
- **Challenge**: Learn from sparse rewards without explicit path planning

---

## 2. Theoretical Background

### 2.1 Reinforcement Learning Basics

**Key Concepts:**
- **State (s)**: Robot's observation of the environment (LiDAR readings + goal direction)
- **Action (a)**: Movement command sent to the robot
- **Reward (r)**: Feedback signal indicating action quality
- **Policy (π)**: Strategy mapping states to actions
- **Q-value Q(s,a)**: Expected cumulative reward for taking action a in state s

### 2.2 Deep Q-Network (DQN)

DQN uses a neural network to approximate the Q-function:

```
Q(s, a; θ) ≈ Q*(s, a)
```

**Key Components:**

1. **Experience Replay**: Store transitions (s, a, r, s') in a buffer and sample random batches
2. **Target Network**: Separate network for stable Q-value targets
3. **ε-Greedy Exploration**: Balance exploration vs exploitation

**Bellman Update:**
```
Q(s, a) ← Q(s, a) + α[r + γ max Q(s', a') - Q(s, a)]
```

Where:
- α = learning rate
- γ = discount factor (0-1)
- r = immediate reward
- s' = next state

### 2.3 Why DQN for Robot Navigation?

- Handles high-dimensional sensor data (LiDAR)
- Learns end-to-end control without hand-crafted features
- Discovers non-obvious navigation strategies
- Generalizes to unseen environments

---

## 3. Environment Setup

### 3.1 Prerequisites

Ensure you have the following installed:

```bash
# Check ROS2 Humble installation
ros2 --version

# Check if TurtleBot3 packages are installed
ros2 pkg list | grep turtlebot3

# Required Python packages
pip3 install scikit-learn numpy matplotlib
```

### 3.2 TurtleBot3 Setup

```bash
# Install TurtleBot3 packages if not already installed
sudo apt install ros-humble-turtlebot3*

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### 3.3 Create Workspace

```bash
# Create and build workspace
mkdir -p ~/dqn_navigation_ws/src
cd ~/dqn_navigation_ws/src

# Create package
ros2 pkg create --build-type ament_python dqn_robot_nav \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs

cd ~/dqn_navigation_ws
colcon build
source install/setup.bash
```

### 3.4 Test Gazebo Simulation

```bash
# Terminal 1: Launch Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Check topics
ros2 topic list
ros2 topic echo /scan --once
```

**Expected topics:**
- `/scan` - LiDAR data
- `/cmd_vel` - Velocity commands
- `/odom` - Odometry data

---

## 4. System Architecture

### 4.1 Component Overview

```
┌─────────────────────────────────────────────────────────┐
│                    DQN Training System                   │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────┐         ┌─────────────────┐          │
│  │   Gazebo     │  scan   │  DQN Agent Node │          │
│  │  Simulation  │────────>│                 │          │
│  │ (TurtleBot3) │<────────│  • State Proc   │          │
│  └──────────────┘ cmd_vel │  • Q-Network    │          │
│         │                  │  • Experience   │          │
│         │ odom             │    Replay       │          │
│         └─────────────────>│  • Training     │          │
│                            └─────────────────┘          │
│                                     │                    │
│                                     v                    │
│                            ┌─────────────────┐          │
│                            │  Data Logger    │          │
│                            │  • Rewards      │          │
│                            │  • Q-values     │          │
│                            │  • Success rate │          │
│                            └─────────────────┘          │
└─────────────────────────────────────────────────────────┘
```

### 4.2 Data Flow

1. **Perception**: Subscribe to `/scan` topic for LiDAR data
2. **State Processing**: Convert raw LiDAR to state representation
3. **Action Selection**: Q-network chooses action (ε-greedy)
4. **Execution**: Publish velocity command to `/cmd_vel`
5. **Feedback**: Receive next state and compute reward
6. **Learning**: Store experience and train Q-network

---

## 5. Implementation

### 5.1 Project Structure

```
dqn_robot_nav/
├── dqn_robot_nav/
│   ├── __init__.py
│   ├── dqn_agent.py        # DQN algorithm implementation
│   ├── environment.py      # ROS2 environment wrapper
│   ├── state_processor.py  # LiDAR data processing
│   ├── train_node.py       # Main training node
│   └── test_node.py        # Evaluation node
├── config/
│   └── training_params.yaml
├── launch/
│   └── train_dqn.launch.py
├── package.xml
└── setup.py
```

### 5.2 State Processor (`state_processor.py`)

The state processor converts raw LiDAR data into a compact representation suitable for the Q-network.

```python
import numpy as np
from typing import Tuple

class StateProcessor:
    """Process LiDAR data into state representation for DQN"""
    
    def __init__(self, n_lidar_bins: int = 10):
        """
        Args:
            n_lidar_bins: Number of bins to discretize 360° LiDAR scan
        """
        self.n_lidar_bins = n_lidar_bins
        self.max_lidar_range = 3.5  # TurtleBot3 LiDAR max range
        
    def process_lidar(self, scan_data: list) -> np.ndarray:
        """
        Process 360-point LiDAR scan into n bins
        
        Args:
            scan_data: List of distance measurements (360 points)
            
        Returns:
            Array of shape (n_lidar_bins,) with min distances per sector
        """
        scan_array = np.array(scan_data)
        
        # Replace inf values with max range
        scan_array[np.isinf(scan_array)] = self.max_lidar_range
        scan_array[np.isnan(scan_array)] = self.max_lidar_range
        
        # Clip values to [0, max_range]
        scan_array = np.clip(scan_array, 0, self.max_lidar_range)
        
        # Divide 360° into bins and take minimum distance in each
        points_per_bin = len(scan_array) // self.n_lidar_bins
        binned_scan = []
        
        for i in range(self.n_lidar_bins):
            start_idx = i * points_per_bin
            end_idx = (i + 1) * points_per_bin if i < self.n_lidar_bins - 1 else len(scan_array)
            bin_min = np.min(scan_array[start_idx:end_idx])
            binned_scan.append(bin_min)
        
        # Normalize to [0, 1]
        return np.array(binned_scan) / self.max_lidar_range
    
    def compute_goal_info(self, 
                         current_pos: Tuple[float, float],
                         goal_pos: Tuple[float, float],
                         current_yaw: float) -> np.ndarray:
        """
        Compute goal distance and relative angle
        
        Args:
            current_pos: (x, y) current position
            goal_pos: (x, y) goal position
            current_yaw: Current heading angle (radians)
            
        Returns:
            Array [distance_to_goal, angle_to_goal] normalized
        """
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        
        # Distance to goal
        distance = np.sqrt(dx**2 + dy**2)
        
        # Angle to goal relative to robot's heading
        angle_to_goal = np.arctan2(dy, dx)
        relative_angle = angle_to_goal - current_yaw
        
        # Normalize angle to [-π, π]
        relative_angle = np.arctan2(np.sin(relative_angle), np.cos(relative_angle))
        
        # Normalize values
        distance_norm = np.clip(distance / 10.0, 0, 1)  # Assume max distance of 10m
        angle_norm = relative_angle / np.pi  # [-1, 1]
        
        return np.array([distance_norm, angle_norm])
    
    def get_state(self,
                  scan_data: list,
                  current_pos: Tuple[float, float],
                  goal_pos: Tuple[float, float],
                  current_yaw: float) -> np.ndarray:
        """
        Combine LiDAR and goal information into complete state
        
        Returns:
            State vector of shape (n_lidar_bins + 2,)
        """
        lidar_state = self.process_lidar(scan_data)
        goal_state = self.compute_goal_info(current_pos, goal_pos, current_yaw)
        
        return np.concatenate([lidar_state, goal_state])
```

**Key Design Choices:**

1. **LiDAR Binning**: Reduces 360 points to 10 bins (36° sectors)
   - Reduces state dimensionality
   - Captures obstacle distribution around robot
   - Each bin contains minimum distance in that sector

2. **Goal Information**: Distance and angle to goal
   - Helps robot know where to navigate
   - Normalized to improve learning stability

3. **Normalization**: All values in [0, 1] or [-1, 1]
   - Improves neural network training
   - Prevents feature domination

### 5.3 DQN Agent (`dqn_agent.py`)

```python
import numpy as np
from sklearn.neural_network import MLPRegressor
from collections import deque
import random
import pickle

class DQNAgent:
    """Deep Q-Network agent using sklearn's MLPRegressor"""
    
    def __init__(self,
                 state_size: int,
                 action_size: int,
                 learning_rate: float = 0.001,
                 gamma: float = 0.99,
                 epsilon: float = 1.0,
                 epsilon_min: float = 0.01,
                 epsilon_decay: float = 0.995,
                 memory_size: int = 10000,
                 batch_size: int = 64,
                 target_update_freq: int = 100):
        """
        Initialize DQN Agent
        
        Args:
            state_size: Dimension of state space
            action_size: Number of discrete actions
            learning_rate: Learning rate for Q-network
            gamma: Discount factor
            epsilon: Initial exploration rate
            epsilon_min: Minimum exploration rate
            epsilon_decay: Decay rate for epsilon
            memory_size: Size of experience replay buffer
            batch_size: Batch size for training
            target_update_freq: Steps between target network updates
        """
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.target_update_freq = target_update_freq
        self.step_count = 0
        
        # Experience replay memory
        self.memory = deque(maxlen=memory_size)
        
        # Q-network (main network)
        self.q_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,  # We'll call partial_fit
            warm_start=True,
            random_state=42
        )
        
        # Target network
        self.target_network = MLPRegressor(
            hidden_layer_sizes=(128, 128),
            activation='relu',
            solver='adam',
            learning_rate_init=learning_rate,
            max_iter=1,
            warm_start=True,
            random_state=42
        )
        
        # Initialize networks with dummy data
        dummy_X = np.random.randn(1, state_size)
        dummy_y = np.random.randn(1, action_size)
        self.q_network.fit(dummy_X, dummy_y)
        self.target_network.fit(dummy_X, dummy_y)
        
    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay memory"""
        self.memory.append((state, action, reward, next_state, done))
    
    def act(self, state: np.ndarray, training: bool = True) -> int:
        """
        Select action using ε-greedy policy
        
        Args:
            state: Current state
            training: If True, use epsilon-greedy; else greedy
            
        Returns:
            Action index
        """
        # Exploration
        if training and np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        
        # Exploitation
        state_reshaped = state.reshape(1, -1)
        q_values = self.q_network.predict(state_reshaped)[0]
        return np.argmax(q_values)
    
    def replay(self) -> float:
        """
        Train Q-network on batch from experience replay
        
        Returns:
            Average loss for the batch
        """
        if len(self.memory) < self.batch_size:
            return 0.0
        
        # Sample random batch
        minibatch = random.sample(self.memory, self.batch_size)
        
        states = np.array([transition[0] for transition in minibatch])
        actions = np.array([transition[1] for transition in minibatch])
        rewards = np.array([transition[2] for transition in minibatch])
        next_states = np.array([transition[3] for transition in minibatch])
        dones = np.array([transition[4] for transition in minibatch])
        
        # Current Q-values
        current_q_values = self.q_network.predict(states)
        
        # Next Q-values from target network
        next_q_values = self.target_network.predict(next_states)
        
        # Compute target Q-values
        target_q_values = current_q_values.copy()
        for i in range(self.batch_size):
            if dones[i]:
                target_q_values[i][actions[i]] = rewards[i]
            else:
                target_q_values[i][actions[i]] = rewards[i] + self.gamma * np.max(next_q_values[i])
        
        # Train network
        self.q_network.partial_fit(states, target_q_values)
        
        # Update target network
        self.step_count += 1
        if self.step_count % self.target_update_freq == 0:
            self.update_target_network()
        
        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        
        # Compute loss (MSE)
        loss = np.mean((target_q_values - current_q_values) ** 2)
        return loss
    
    def update_target_network(self):
        """Copy weights from Q-network to target network"""
        # In sklearn, we copy the entire model
        self.target_network = pickle.loads(pickle.dumps(self.q_network))
        print("Target network updated")
    
    def save(self, filepath: str):
        """Save model to file"""
        model_data = {
            'q_network': self.q_network,
            'target_network': self.target_network,
            'epsilon': self.epsilon,
            'step_count': self.step_count
        }
        with open(filepath, 'wb') as f:
            pickle.dump(model_data, f)
        print(f"Model saved to {filepath}")
    
    def load(self, filepath: str):
        """Load model from file"""
        with open(filepath, 'rb') as f:
            model_data = pickle.load(f)
        self.q_network = model_data['q_network']
        self.target_network = model_data['target_network']
        self.epsilon = model_data['epsilon']
        self.step_count = model_data['step_count']
        print(f"Model loaded from {filepath}")
```

**Implementation Notes:**

1. **sklearn MLPRegressor**: Simple but effective for small-scale problems
   - Easy to use, no GPU required
   - Good for educational purposes
   - Hidden layers: [128, 128] neurons

2. **Experience Replay**: Breaks temporal correlation
   - Stores last 10,000 experiences
   - Samples random batches for training

3. **Target Network**: Stabilizes training
   - Updated every 100 steps
   - Prevents moving target problem

4. **ε-Greedy**: Balances exploration/exploitation
   - Starts at 1.0 (100% random)
   - Decays to 0.01 (1% random)

### 5.4 Environment Wrapper (`environment.py`)

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import numpy as np
from typing import Tuple
import math

class TurtleBot3Env(Node):
    """ROS2 Environment wrapper for TurtleBot3 navigation"""
    
    def __init__(self):
        super().__init__('turtlebot3_env')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', 
                                                  self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom',
                                                  self.odom_callback, 10)
        
        # Gazebo service for resetting world
        self.reset_world_client = self.create_client(Empty, '/reset_world')
        
        # State variables
        self.scan_data = None
        self.position = (0.0, 0.0)
        self.yaw = 0.0
        self.last_position = (0.0, 0.0)
        
        # Goal position (will be randomized)
        self.goal_position = (4.0, 4.0)
        
        # Action space: 5 discrete actions
        self.actions = {
            0: (0.15, 0.0),    # Forward
            1: (0.0, 0.5),     # Rotate left
            2: (0.0, -0.5),    # Rotate right
            3: (0.08, 0.3),    # Forward + left
            4: (0.08, -0.3),   # Forward + right
        }
        
        self.collision_threshold = 0.2  # meters
        self.goal_threshold = 0.3       # meters
        
    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan"""
        self.scan_data = list(msg.ranges)
    
    def odom_callback(self, msg: Odometry):
        """Store latest odometry data"""
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + 
                        orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + 
                            orientation_q.z * orientation_q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def step(self, action: int) -> Tuple[np.ndarray, float, bool]:
        """
        Execute action and return next state, reward, done
        
        Args:
            action: Action index
            
        Returns:
            next_state, reward, done
        """
        # Execute action
        linear_vel, angular_vel = self.actions[action]
        self.send_velocity(linear_vel, angular_vel)
        
        # Wait for state update
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check termination conditions
        done = False
        reward = 0.0
        
        # 1. Check collision
        if self.is_collision():
            reward = -100.0
            done = True
            self.get_logger().info("Collision detected!")
            
        # 2. Check goal reached
        elif self.is_goal_reached():
            reward = 200.0
            done = True
            self.get_logger().info("Goal reached!")
            
        # 3. Ongoing reward
        else:
            reward = self.compute_reward(action)
        
        return self.get_state(), reward, done
    
    def compute_reward(self, action: int) -> float:
        """
        Compute reward for ongoing navigation
        
        Reward components:
        1. Progress toward goal (positive)
        2. Proximity to obstacles (negative)
        3. Action penalty (encourage efficiency)
        """
        # Distance to goal
        current_dist = self.distance_to_goal()
        
        # Progress reward (compare to last position if available)
        if hasattr(self, 'last_distance'):
            progress = self.last_distance - current_dist
            progress_reward = progress * 10.0  # Scale factor
        else:
            progress_reward = 0.0
        
        self.last_distance = current_dist
        
        # Obstacle proximity penalty
        min_obstacle_dist = np.min(self.scan_data) if self.scan_data else 3.5
        if min_obstacle_dist < 0.5:
            obstacle_penalty = -5.0 * (0.5 - min_obstacle_dist)
        else:
            obstacle_penalty = 0.0
        
        # Action penalty (encourage forward motion)
        action_penalty = -0.01 if action in [1, 2] else 0.0  # Penalize pure rotation
        
        # Time penalty (encourage faster completion)
        time_penalty = -0.1
        
        total_reward = progress_reward + obstacle_penalty + action_penalty + time_penalty
        
        return total_reward
    
    def is_collision(self) -> bool:
        """Check if robot has collided with obstacle"""
        if self.scan_data is None:
            return False
        min_distance = np.min(self.scan_data)
        return min_distance < self.collision_threshold
    
    def is_goal_reached(self) -> bool:
        """Check if robot has reached goal"""
        return self.distance_to_goal() < self.goal_threshold
    
    def distance_to_goal(self) -> float:
        """Compute Euclidean distance to goal"""
        dx = self.goal_position[0] - self.position[0]
        dy = self.goal_position[1] - self.position[1]
        return math.sqrt(dx**2 + dy**2)
    
    def send_velocity(self, linear: float, angular: float):
        """Send velocity command to robot"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def reset(self, random_goal: bool = True) -> np.ndarray:
        """
        Reset environment for new episode
        
        Args:
            random_goal: If True, randomize goal position
            
        Returns:
            Initial state
        """
        # Stop robot
        self.send_velocity(0.0, 0.0)
        
        # Reset Gazebo world (resets robot and environment to initial state)
        self.reset_world()
        
        # Randomize goal position
        if random_goal:
            self.goal_position = (
                np.random.uniform(-3.5, 3.5),
                np.random.uniform(-3.5, 3.5)
            )
        
        # Wait for state update after reset
        rclpy.spin_once(self, timeout_sec=0.5)
        
        self.last_distance = self.distance_to_goal()
        
        return self.get_state()
    
    def reset_world(self):
        """Reset Gazebo world using /reset_world service"""
        if not self.reset_world_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Reset world service not available')
            return
        
        # Create empty request
        request = Empty.Request()
        
        # Call service
        future = self.reset_world_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            self.get_logger().info('World reset successfully')
        else:
            self.get_logger().error('Failed to reset world')
    
    def get_state(self) -> np.ndarray:
        """Get current state (must be implemented with StateProcessor)"""
        # This will be combined with StateProcessor in the training node
        return None
```

**Environment Design:**

1. **Action Space**: 5 discrete actions
   - Forward movement
   - Pure rotations (left/right)
   - Combined movements (arc trajectories)

2. **Reward Structure**:
   - **Goal reached**: +200
   - **Collision**: -100
   - **Progress**: +10 per meter toward goal
   - **Obstacle proximity**: -5 when too close
   - **Time penalty**: -0.1 per step (encourages efficiency)

3. **Termination Conditions**:
   - Collision (distance < 0.2m)
   - Goal reached (distance < 0.3m)
   - Maximum steps (defined in training loop)

### 5.5 Training Node (`train_node.py`)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor
import matplotlib.pyplot as plt
from datetime import datetime
import os

class DQNTrainingNode(Node):
    """Main training node for DQN navigation"""
    
    def __init__(self):
        super().__init__('dqn_training_node')
        
        # Training parameters
        self.n_episodes = 500
        self.max_steps_per_episode = 500
        self.state_size = 12  # 10 LiDAR bins + 2 goal info
        self.action_size = 5
        
        # Initialize components
        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=10)
        self.agent = DQNAgent(
            state_size=self.state_size,
            action_size=self.action_size,
            learning_rate=0.001,
            gamma=0.99,
            epsilon=1.0,
            epsilon_min=0.01,
            epsilon_decay=0.995,
            memory_size=10000,
            batch_size=64
        )
        
        # Logging
        self.episode_rewards = []
        self.episode_steps = []
        self.success_count = 0
        self.collision_count = 0
        
        # Create results directory
        self.results_dir = f"results_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.results_dir, exist_ok=True)
        
    def get_processed_state(self):
        """Get processed state from environment"""
        return self.state_processor.get_state(
            self.env.scan_data,
            self.env.position,
            self.env.goal_position,
            self.env.yaw
        )
    
    def train(self):
        """Main training loop"""
        self.get_logger().info("Starting DQN training...")
        
        for episode in range(self.n_episodes):
            # Reset environment
            self.env.reset(random_goal=True)
            rclpy.spin_once(self.env, timeout_sec=0.5)
            
            state = self.get_processed_state()
            episode_reward = 0
            
            for step in range(self.max_steps_per_episode):
                # Select and execute action
                action = self.agent.act(state, training=True)
                next_state_raw, reward, done = self.env.step(action)
                next_state = self.get_processed_state()
                
                # Store experience
                self.agent.remember(state, action, reward, next_state, done)
                
                # Train agent
                if len(self.agent.memory) > self.agent.batch_size:
                    loss = self.agent.replay()
                
                episode_reward += reward
                state = next_state
                
                if done:
                    break
                
                # Prevent blocking
                rclpy.spin_once(self.env, timeout_sec=0.01)
            
            # Episode statistics
            self.episode_rewards.append(episode_reward)
            self.episode_steps.append(step + 1)
            
            if self.env.is_goal_reached():
                self.success_count += 1
            if self.env.is_collision():
                self.collision_count += 1
            
            # Logging
            if episode % 10 == 0:
                avg_reward = np.mean(self.episode_rewards[-10:])
                success_rate = self.success_count / (episode + 1) * 100
                
                self.get_logger().info(
                    f"Episode: {episode}/{self.n_episodes} | "
                    f"Steps: {step+1} | "
                    f"Reward: {episode_reward:.2f} | "
                    f"Avg Reward (10): {avg_reward:.2f} | "
                    f"Epsilon: {self.agent.epsilon:.3f} | "
                    f"Success Rate: {success_rate:.1f}%"
                )
            
            # Save model periodically
            if episode % 50 == 0 and episode > 0:
                model_path = os.path.join(self.results_dir, f"model_episode_{episode}.pkl")
                self.agent.save(model_path)
        
        # Final save
        self.agent.save(os.path.join(self.results_dir, "model_final.pkl"))
        self.plot_results()
        
    def plot_results(self):
        """Plot training curves"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # Episode rewards
        axes[0, 0].plot(self.episode_rewards)
        axes[0, 0].set_title('Episode Rewards')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Total Reward')
        axes[0, 0].grid(True)
        
        # Moving average reward
        window = 20
        if len(self.episode_rewards) >= window:
            moving_avg = np.convolve(self.episode_rewards, 
                                    np.ones(window)/window, mode='valid')
            axes[0, 1].plot(moving_avg)
            axes[0, 1].set_title(f'Moving Average Reward (window={window})')
            axes[0, 1].set_xlabel('Episode')
            axes[0, 1].set_ylabel('Avg Reward')
            axes[0, 1].grid(True)
        
        # Episode steps
        axes[1, 0].plot(self.episode_steps)
        axes[1, 0].set_title('Steps per Episode')
        axes[1, 0].set_xlabel('Episode')
        axes[1, 0].set_ylabel('Steps')
        axes[1, 0].grid(True)
        
        # Success rate
        window = 50
        success_history = []
        for i in range(len(self.episode_rewards)):
            if i < window:
                success_history.append(self.success_count / (i + 1))
            else:
                # Count successes in last 'window' episodes
                recent_successes = sum([1 for j in range(i-window+1, i+1) 
                                       if self.episode_rewards[j] > 100])
                success_history.append(recent_successes / window)
        
        axes[1, 1].plot([s * 100 for s in success_history])
        axes[1, 1].set_title(f'Success Rate (window={window})')
        axes[1, 1].set_xlabel('Episode')
        axes[1, 1].set_ylabel('Success Rate (%)')
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.results_dir, 'training_results.png'))
        self.get_logger().info(f"Results saved to {self.results_dir}")

def main(args=None):
    rclpy.init(args=args)
    trainer = DQNTrainingNode()
    
    try:
        trainer.train()
    except KeyboardInterrupt:
        trainer.get_logger().info("Training interrupted by user")
    finally:
        trainer.env.send_velocity(0.0, 0.0)
        trainer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 6. Training Process

### 6.1 Launch Training

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start training
cd ~/dqn_navigation_ws
source install/setup.bash
ros2 run dqn_robot_nav train_node
```

### 6.2 Expected Training Progression

**Phase 1: Random Exploration (Episodes 0-50)**
- Agent explores randomly (ε ≈ 1.0)
- Frequent collisions
- Occasional lucky goal reaches
- Average reward: -50 to 0

**Phase 2: Learning Basic Behaviors (Episodes 50-150)**
- Agent learns to avoid immediate obstacles
- Starts moving toward goal occasionally
- ε decays to ~0.6
- Average reward: 0 to 50

**Phase 3: Policy Refinement (Episodes 150-350)**
- Consistent obstacle avoidance
- More efficient paths
- ε decays to ~0.2
- Average reward: 50 to 100
- Success rate: 30-50%

**Phase 4: Convergence (Episodes 350-500)**
- Stable policy
- High success rate (>60%)
- Smooth navigation
- Average reward: 100-150

### 6.3 Monitoring Training

Watch for these metrics in the console output:

```
Episode: 100/500 | Steps: 234 | Reward: 87.45 | Avg Reward (10): 75.32 | 
Epsilon: 0.605 | Success Rate: 35.0%
```

**Good signs:**
- Average reward increasing
- Success rate growing
- Steps per episode decreasing (more efficient)
- Epsilon decaying smoothly

**Warning signs:**
- Reward stuck or decreasing
- Very high collision rate (>70%)
- Agent getting stuck in corners
- Training instability (wild reward swings)

---

## 7. Evaluation and Testing

### 7.1 Test Node (`test_node.py`)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dqn_robot_nav.dqn_agent import DQNAgent
from dqn_robot_nav.environment import TurtleBot3Env
from dqn_robot_nav.state_processor import StateProcessor
import numpy as np

class DQNTestNode(Node):
    """Test trained DQN agent"""
    
    def __init__(self, model_path: str):
        super().__init__('dqn_test_node')
        
        self.state_size = 12
        self.action_size = 5
        
        # Load trained agent
        self.agent = DQNAgent(self.state_size, self.action_size)
        self.agent.load(model_path)
        self.agent.epsilon = 0.0  # Greedy policy only
        
        self.env = TurtleBot3Env()
        self.state_processor = StateProcessor(n_lidar_bins=10)
        
        self.get_logger().info(f"Loaded model from {model_path}")
    
    def get_processed_state(self):
        return self.state_processor.get_state(
            self.env.scan_data,
            self.env.position,
            self.env.goal_position,
            self.env.yaw
        )
    
    def test(self, n_episodes: int = 10):
        """Run test episodes"""
        successes = 0
        total_rewards = []
        
        for episode in range(n_episodes):
            self.env.reset(random_goal=True)
            rclpy.spin_once(self.env, timeout_sec=0.5)
            
            state = self.get_processed_state()
            episode_reward = 0
            
            for step in range(500):
                action = self.agent.act(state, training=False)
                _, reward, done = self.env.step(action)
                state = self.get_processed_state()
                
                episode_reward += reward
                
                if done:
                    if self.env.is_goal_reached():
                        successes += 1
                        self.get_logger().info(
                            f"Episode {episode+1}: SUCCESS! "
                            f"Steps: {step+1}, Reward: {episode_reward:.2f}"
                        )
                    else:
                        self.get_logger().info(
                            f"Episode {episode+1}: COLLISION. "
                            f"Steps: {step+1}, Reward: {episode_reward:.2f}"
                        )
                    break
                
                rclpy.spin_once(self.env, timeout_sec=0.01)
            
            total_rewards.append(episode_reward)
        
        # Print statistics
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f"Test Results over {n_episodes} episodes:")
        self.get_logger().info(f"Success Rate: {successes/n_episodes*100:.1f}%")
        self.get_logger().info(f"Avg Reward: {np.mean(total_rewards):.2f}")
        self.get_logger().info(f"Std Reward: {np.std(total_rewards):.2f}")
        self.get_logger().info("="*50)

def main(args=None):
    import sys
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run dqn_robot_nav test_node <model_path>")
        return
    
    model_path = sys.argv[1]
    tester = DQNTestNode(model_path)
    
    try:
        tester.test(n_episodes=10)
    except KeyboardInterrupt:
        pass
    finally:
        tester.env.send_velocity(0.0, 0.0)
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7.2 Running Tests

```bash
# Test trained model
ros2 run dqn_robot_nav test_node results_YYYYMMDD_HHMMSS/model_final.pkl
```

### 7.3 Evaluation Metrics

1. **Success Rate**: Percentage of episodes where goal is reached
   - Target: >60% after full training

2. **Average Reward**: Mean total reward per episode
   - Higher is better (good policies: >100)

3. **Average Steps**: Efficiency of navigation
   - Lower indicates more direct paths

4. **Collision Rate**: Percentage of collision episodes
   - Should decrease over training

---

## 8. Troubleshooting

### 8.1 Common Issues

**Problem: Robot doesn't move**
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Verify Gazebo is running
- Check action execution in environment

**Problem: No LiDAR data**
- Check `/scan` topic: `ros2 topic echo /scan`
- Verify TurtleBot3 model: `echo $TURTLEBOT3_MODEL`
- Restart Gazebo simulation

**Problem: Training is unstable**
- Reduce learning rate (try 0.0005)
- Increase batch size (try 128)
- Check reward scaling
- Verify state normalization

**Problem: Agent always collides**
- Check collision threshold (might be too small)
- Increase obstacle penalty in reward
- Check LiDAR binning (try more bins)
- Increase exploration (higher initial ε)

**Problem: Agent spins in place**
- Add rotation penalty in reward function
- Reduce angular velocity in actions
- Check goal information in state

### 8.2 Debugging Tips

1. **Visualize State**: Print state values to understand what agent sees
```python
print(f"LiDAR bins: {state[:10]}")
print(f"Goal info: {state[10:]}")
```

2. **Log Q-values**: See what agent is thinking
```python
q_values = self.agent.q_network.predict(state.reshape(1, -1))
print(f"Q-values: {q_values}")
```

3. **Record Episodes**: Save video of robot behavior
```bash
ros2 run image_view video_recorder image:=/camera/image_raw
```

4. **Plot Trajectories**: Visualize robot paths
- Log positions during episode
- Plot in matplotlib

---

## 9. Extensions and Challenges

### 9.1 Easy Extensions

1. **Add More Actions**
   - Backward movement
   - Different speeds
   - Combined actions

2. **Modify Reward Function**
   - Add smoothness reward (penalize jerky motion)
   - Reward for maintaining safe distance from walls
   - Time bonus for fast completion

3. **Change Environment**
   - Use different Gazebo worlds
   - Add dynamic obstacles
   - Multiple goal positions

### 9.2 Intermediate Challenges

1. **Improve State Representation**
   - Add velocity information
   - Include previous actions
   - Use history of LiDAR scans

2. **Curriculum Learning**
   - Start with close goals
   - Gradually increase difficulty
   - Add obstacles progressively

3. **Better Network Architecture**
   - Replace sklearn with PyTorch/TensorFlow
   - Add convolutional layers for LiDAR
   - Implement Dueling DQN

### 9.3 Advanced Challenges

1. **Multi-Goal Navigation**
   - Sequence of waypoints
   - Path planning integration
   - Dynamic goal switching

2. **Transfer Learning**
   - Train in one world, test in another
   - Domain randomization
   - Sim-to-real transfer

3. **Multi-Agent System**
   - Multiple robots
   - Cooperative navigation
   - Collision avoidance between robots

4. **Real Robot Deployment**
   - Deploy to physical TurtleBot3
   - Handle sensor noise
   - Real-time performance optimization

---

## 10. Lab Questions and Analysis

### 10.1 Understanding Questions

1. **What is the role of the discount factor γ?**
   - How does changing γ from 0.9 to 0.99 affect learning?

2. **Why do we need experience replay?**
   - What happens if you train on consecutive experiences?

3. **What is the target network for?**
   - Try training without target network updates. What happens?

4. **How does ε-greedy exploration work?**
   - Plot exploration vs. exploitation over training

### 10.2 Experimental Tasks

1. **Reward Shaping Study**
   - Try different reward functions
   - Compare convergence speed and final performance
   - Which components are most important?

2. **Hyperparameter Tuning**
   - Test different learning rates: [0.0001, 0.001, 0.01]
   - Test different batch sizes: [32, 64, 128]
   - Test different network sizes: [(64,64), (128,128), (256,256)]
   - Document results in a table

3. **State Representation Impact**
   - Compare 5, 10, and 20 LiDAR bins
   - Try removing goal information
   - Add robot velocity to state

### 10.3 Report Requirements

Your lab report should include:

1. **Implementation** (30%):
   - Working code with comments
   - All files properly organized
   - Successful training run

2. **Results** (30%):
   - Training curves (rewards, success rate)
   - Test performance metrics
   - Example trajectories

3. **Analysis** (30%):
   - Answer understanding questions
   - Experimental task results
   - Comparison of different configurations
   - Discussion of what worked and what didn't

4. **Conclusion** (10%):
   - Lessons learned
   - Limitations of approach
   - Potential improvements

---

## 11. Additional Resources

### 11.1 Key Papers

1. **DQN Original Paper**: Mnih et al. "Playing Atari with Deep Reinforcement Learning" (2013)
2. **DQN Nature Paper**: Mnih et al. "Human-level control through deep reinforcement learning" (2015)
3. **Double DQN**: van Hasselt et al. "Deep Reinforcement Learning with Double Q-learning" (2015)

### 11.2 Useful Links

- ROS2 Documentation: https://docs.ros.org/en/humble/
- TurtleBot3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- Reinforcement Learning Book (Sutton & Barto): http://incompleteideas.net/book/
- OpenAI Spinning Up: https://spinningup.openai.com/

### 11.3 Tips for Success

1. **Start Simple**: Get basic version working before adding complexity
2. **Debug Incrementally**: Test each component separately
3. **Visualize Everything**: Plot states, rewards, Q-values
4. **Be Patient**: RL training takes time and tuning
5. **Ask for Help**: Discuss with classmates and instructors

---

## Appendix A: Setup Script

Save this as `setup_dqn_lab.sh`:

```bash
#!/bin/bash

# DQN Robot Navigation Lab Setup Script

echo "Setting up DQN Robot Navigation Lab..."

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

# Create workspace
mkdir -p ~/dqn_navigation_ws/src
cd ~/dqn_navigation_ws/src

# Create package
ros2 pkg create --build-type ament_python dqn_robot_nav \
  --dependencies rclpy std_msgs geometry_msgs sensor_msgs nav_msgs gazebo_msgs

# Create directory structure
cd dqn_robot_nav
mkdir -p dqn_robot_nav config launch

# Install Python dependencies
pip3 install scikit-learn numpy matplotlib

# Build workspace
cd ~/dqn_navigation_ws
colcon build
source install/setup.bash

echo "Setup complete!"
echo "Next steps:"
echo "1. Copy the Python files to ~/dqn_navigation_ws/src/dqn_robot_nav/dqn_robot_nav/"
echo "2. Update setup.py with entry points"
echo "3. Rebuild: colcon build"
echo "4. Source: source install/setup.bash"
echo "5. Launch Gazebo: ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "6. Start training: ros2 run dqn_robot_nav train_node"
```

---

## Appendix B: Sample setup.py

```python
from setuptools import setup

package_name = 'dqn_robot_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='DQN for robot navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'train_node = dqn_robot_nav.train_node:main',
            'test_node = dqn_robot_nav.test_node:main',
        ],
    },
)
```

---

**Good luck with your lab! Remember: RL is about iteration and experimentation. Don't be discouraged by initial failures – they're part of the learning process!**