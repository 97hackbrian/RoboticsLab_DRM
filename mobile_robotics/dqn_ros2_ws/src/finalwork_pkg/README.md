# DQN Navigation - finalwork_pkg

Deep Q-Learning Navigation system for autonomous robot navigation in complex environments using ROS2 Jazzy and Stage simulator.

## 📋 Overview

This package implements a **Deep Q-Network (DQN)** agent with **Curriculum Learning** for autonomous robot navigation. The robot learns to navigate through obstacle-filled environments by progressively increasing task difficulty, achieving up to **65% success rate** in reaching random goals while avoiding collisions.

### Key Features

- ✅ **Curriculum Learning:** Progressive difficulty from easy (2-5m) to expert (10-20m) goals
- ✅ **Prioritized Experience Replay (PER):** Learn 3x more from critical experiences
- ✅ **Double DQN:** Prevents Q-value overestimation
- ✅ **Front-Focused Vision:** 100° FOV LIDAR for focused obstacle avoidance
- ✅ **Adaptive Reward Shaping:** Gradual proximity penalties and progress rewards
- ✅ **Checkpoint System:** Auto-save every 50 episodes with recovery support

---

## 🎯 Architecture

```
State Space: 22 dimensions
├─ LIDAR sectors [0:20]: Front arc obstacle distances (100° FOV)
├─ Goal distance [20]: Normalized distance to target
└─ Goal angle [21]: Normalized angle to target

Action Space: 3 discrete actions
├─ 0: Turn LEFT  (linear=0.25, angular=+1.2)
├─ 1: Turn RIGHT (linear=0.25, angular=-1.2)
└─ 2: GO FORWARD (linear=0.4, angular=0.0)

Neural Network: MLPRegressor (scikit-learn)
└─ Architecture: 256 → 256 → 128 → 3 (Q-values)
```

---

## 🚀 Installation

### Prerequisites

- Ubuntu 22.04 (recommended)
- ROS2 Jazzy
- Python 3.10+

### 1. Install ROS2 Jazzy

Follow official installation guide:
```bash
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
```

### 2. Install Stage Simulator

#### Install from GitHub (Required)

```bash
cd ~/
git clone https://github.com/rtv/Stage.git
cd Stage
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

#### Install stage_ros2

```bash
cd ~/dqn_ros2_ws/src
git clone https://github.com/tuw-robotics/stage_ros2.git
```

### 3. Install Python Dependencies

```bash
pip install numpy scikit-learn
```

### 4. Clone This Package

```bash
cd ~/dqn_ros2_ws/src
git clone <your-repo-url> finalwork_pkg
```

---

## 🔨 Compilation

### Build the Package

```bash
cd ~/dqn_ros2_ws
colcon build --packages-select finalwork_pkg
source install/setup.bash
```

### Verify Installation

```bash
ros2 pkg list | grep finalwork
# Should output: finalwork-pkg

ros2 launch finalwork_pkg --show-args
# Should list available launch files
```

---

## 🎮 Usage

### Training Mode

Train the robot with curriculum learning:

```bash
ros2 launch finalwork_pkg train_launch.py
```

#### Training Parameters

```bash
ros2 launch finalwork_pkg train_launch.py \
  num_episodes:=1000 \
  save_interval:=50 \
  model_name:=dqn_model.pkl \
  world:=cave.world
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_episodes` | 1000 | Total training episodes |
| `save_interval` | 50 | Save checkpoint every N episodes |
| `model_name` | `dqn_model.pkl` | Model filename |
| `world` | `cave.world` | Stage world file |

#### Curriculum Phases

The robot automatically progresses through difficulty levels:

- **Phase 1 (Eps 0-250):** Goals 2-5m → Learn basic movement
- **Phase 2 (Eps 250-500):** Goals 5-10m → Learn obstacle avoidance
- **Phase 3 (Eps 500-750):** Goals 10-15m → Complex navigation
- **Phase 4 (Eps 750+):** Goals 10-20m → Expert navigation

---

### Testing Mode

Test a trained model with a specific goal:

```bash
ros2 launch finalwork_pkg test_launch.py \
  model_path:=/path/to/model.pkl \
  goal_x:=5.0 \
  goal_y:=3.0
```

#### Testing Parameters

| Parameter | Required | Description |
|-----------|----------|-------------|
| `model_path` | ✅ Yes | Path to trained model |
| `goal_x` | ✅ Yes | Goal X coordinate |
| `goal_y` | ✅ Yes | Goal Y coordinate |
| `num_episodes` | ❌ No | Test episodes (default: 1) |
| `max_steps` | ❌ No | Max steps per episode (default: 500) |

---

### Continue Training from Checkpoint

Resume training from a saved checkpoint:

```bash
# Train initial 200 episodes
ros2 launch finalwork_pkg train_launch.py num_episodes:=200

# Continue from episode 200
ros2 launch finalwork_pkg train_launch.py \
  model_name:=dqn_model_ep200.pkl \
  num_episodes:=500
```

The system will:
1. Load the model
2. Restore epsilon, memory, and episode number
3. Resume from episode 201

---

## 📊 Monitoring Training

### Real-time Logs

Training outputs summary every episode:

```
Ep 111/1000 | Steps: 33 | Reward: 281.36 | Avg100: 139.90 
| ε: 0.004 | Goals: 72 (64.9%) | Time: 10.5s
```

| Metric | Description |
|--------|-------------|
| `Ep` | Current episode / Total episodes |
| `Steps` | Steps taken in this episode |
| `Reward` | Total reward for this episode |
| `Avg100` | Average reward over last 100 episodes |
| `ε` | Current exploration rate |
| `Goals` | Total goals reached (success rate) |
| `Time` | Episode duration |

### Saved Checkpoints

Models are saved in `models/` directory:

```
models/
├── dqn_model.pkl           # Latest model
├── dqn_model_ep50.pkl      # Checkpoint at episode 50
├── dqn_model_ep100.pkl     # Checkpoint at episode 100
├── dqn_model_ep150.pkl     # Checkpoint at episode 150
└── ...
```

---

## 🔧 Configuration

### Modify Hyperparameters

Edit `finalwork_pkg/dqn_agent.py`:

```python
# Learning parameters
GAMMA = 0.99                # Discount factor
EPSILON_MIN = 0.001         # Minimum exploration
EPSILON_DECAY = 0.9985      # Exploration decay rate
LEARNING_RATE = 0.0005      # Neural network learning rate
BATCH_SIZE = 256            # Training batch size
MEMORY_SIZE = 70000         # Experience replay buffer size
```

### Modify Reward System

Edit `finalwork_pkg/environment_manager.py`:

```python
# Rewards
REWARD_GOAL = 200.0         # Goal reached
REWARD_COLLISION = -210.0   # Collision penalty
REWARD_TIMEOUT = -14.0      # Timeout penalty

# Behavior rewards
# Progress reward: 25.0 * (prev_dist - curr_dist)
# Proximity penalty: gradual from -5 to -75
```

### Change LIDAR FOV

Edit `finalwork_pkg/state_processor.py`:

```python
LIDAR_FOV = 100.0  # Front arc in degrees (±50°)
```

---

## 🎯 Performance

### Current Results (Episode 111)

- **Success Rate:** 64.9% (72/111 goals reached)
- **Average Reward:** 139.90 (last 100 episodes)
- **Exploration:** ε = 0.004 (using learned policy)

### Expected Results

| Phase | Episodes | Success Rate | Avg Reward |
|-------|----------|--------------|------------|
| Phase 1 | 0-250 | 60-80% | +100 to +200 |
| Phase 2 | 250-500 | 50-70% | +50 to +150 |
| Phase 3 | 500-750 | 40-60% | +20 to +100 |
| Phase 4 | 750+ | 60-70% | +50 to +150 |

---

## 🐛 Troubleshooting

### Stage Crashes Frequently

**Problem:** Stage simulator crashes every ~100-150 episodes (segmentation fault)

**Solutions:**

1. **Train in blocks:**
   ```bash
   # Train 100 episodes
   ros2 launch finalwork_pkg train_launch.py num_episodes:=100
   
   # Load checkpoint and continue
   ros2 launch finalwork_pkg train_launch.py \
     model_name:=dqn_model_ep100.pkl \
     num_episodes:=200
   ```

2. **Increase save frequency:**
   ```bash
   ros2 launch finalwork_pkg train_launch.py save_interval:=25
   ```

### Robot Doesn't Learn

**Check:**

1. Epsilon decay: Should decrease over episodes
2. Memory size: Should fill up (~70k transitions)
3. Reward values: Should show positive trend
4. Success rate: Should increase in Phase 1

### Robot Collides Immediately

**Possible causes:**

1. LIDAR not receiving data
2. Wrong coordinate frame
3. Model corrupted

**Fix:**
```bash
# Check topics
ros2 topic list
ros2 topic echo /scan --once

# Restart with fresh model
ros2 launch finalwork_pkg train_launch.py model_name:=dqn_model_new.pkl
```

---

## 📚 Package Structure

```
finalwork_pkg/
├── finalwork_pkg/
│   ├── __init__.py
│   ├── dqn_agent.py           # DQN algorithm implementation
│   ├── environment_manager.py # ROS2 environment interface
│   ├── state_processor.py     # Sensor data processing
│   ├── train_node.py          # Training node
│   └── test_node.py           # Testing node
├── launch/
│   ├── train_launch.py        # Training launch file
│   └── test_launch.py         # Testing launch file
├── models/                    # Saved models directory
│   └── README.md
├── package.xml                # ROS2 package manifest
├── setup.py                   # Python package setup
└── README.md                  # This file
```

---

## 🔗 Dependencies

### ROS2 Packages

- `rclpy` - ROS2 Python client library
- `std_msgs` - Standard ROS2 messages
- `geometry_msgs` - Twist messages for robot control
- `sensor_msgs` - LaserScan messages
- `nav_msgs` - Odometry messages

### Python Packages

- `numpy` - Numerical operations
- `scikit-learn` - MLPRegressor for neural network

### External

- **Stage:** [github.com/rtv/Stage](https://github.com/rtv/Stage)
- **stage_ros2:** [github.com/tuw-robotics/stage_ros2](https://github.com/tuw-robotics/stage_ros2)

---

## 📖 References

### Deep Q-Learning

- Mnih et al. (2015) - "Human-level control through deep reinforcement learning"
- Van Hasselt et al. (2016) - "Deep Reinforcement Learning with Double Q-learning"
- Schaul et al. (2016) - "Prioritized Experience Replay"

### Curriculum Learning

- Bengio et al. (2009) - "Curriculum Learning"
- Florensa et al. (2017) - "Reverse Curriculum Generation for RL"

---

## 🚀 Future Improvements

### Recommended Upgrades

1. **Migrate to PyTorch:**
   - GPU acceleration (10-50x speedup)
   - Better architectures (Dueling DQN, Rainbow)
   - Advanced techniques (Noisy Networks, Distributional RL)

2. **Hindsight Experience Replay (HER):**
   - Learn from failed episodes
   - 3-10x more data efficiency

3. **Visual inputs:**
   - Add camera for rich perception
   - CNN-based feature extraction

4. **Continuous actions:**
   - Switch to A3C/PPO for smooth control
   - Better maneuvering in tight spaces

---

## 📄 License

MIT Licence

---

## 👥 Authors

Brayan Duran Toconas

Jhon Larico Macha

Victor Gutierrez Kaisler

---

## 🙏 Acknowledgments

- Stage Simulator by Richard Vaughan
- ROS2 Community
- OpenAI Gym for RL inspiration

