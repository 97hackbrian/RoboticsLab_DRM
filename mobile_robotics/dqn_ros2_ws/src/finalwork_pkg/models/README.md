# Models Directory

This directory contains trained DQN models.

## Files

- `dqn_model.pkl` - Final trained model
- `dqn_model_epN.pkl` - Checkpoint at episode N
- `dqn_model_metrics.pkl` - Training metrics (rewards, steps, etc.)

## Usage

To use a trained model:

```bash
ros2 launch finalwork_pkg test_launch.py \
    model_path:=/full/path/to/models/dqn_model.pkl \
    goal_x:=5.0 \
    goal_y:=5.0
```
