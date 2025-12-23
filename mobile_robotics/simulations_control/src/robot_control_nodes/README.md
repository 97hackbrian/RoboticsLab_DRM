# robot_control_nodes

ROS2 package for robot control with ArUco marker tracking, LiDAR obstacle avoidance, and synchronized velocity mixing.

## Overview

This package contains three synchronized nodes that work together to control a robot:

1. **tracking_aruco** - Tracks ArUco markers and publishes angular velocity
2. **lidar_stop** - Monitors LiDAR for obstacles and controls linear velocity
3. **mixer_node** - Synchronizes and mixes velocities with safety limits

## Nodes

### tracking_aruco
- **Subscribes**: `/camera/image_raw` (sensor_msgs/Image)
- **Publishes**: `/cmd_vel_angular` (geometry_msgs/TwistStamped)
- Detects ArUco markers (DICT_4X4_50) and publishes angular velocity commands

### lidar_stop
- **Subscribes**: `/scan` (sensor_msgs/LaserScan)
- **Publishes**: `/cmd_vel_linear` (geometry_msgs/TwistStamped)
- Monitors front sector (±30°) and controls linear velocity to avoid collisions
- Safe distance: 0.5m, Slowdown distance: 1.0m

### mixer_node
- **Subscribes**: `/cmd_vel_linear`, `/cmd_vel_angular` (geometry_msgs/TwistStamped)
- **Publishes**: `/cmd_vel` (geometry_msgs/Twist)
- Uses ApproximateTimeSynchronizer (queue_size=10, slop=0.1s)
- **Safety Limits**:
  - Max linear velocity: 0.5 m/s (configurable)
  - Max angular velocity: 1.0 rad/s (configurable)

## Building

```bash
cd simulations_control
colcon build --packages-select robot_control_nodes
source install/setup.bash
```

## Launch File

### Using Launch File (Recommended)

Launch all three nodes with a single command:

```bash
ros2 launch robot_control_nodes robot_control.launch.py
```

### Custom Parameters

You can override default parameters:

```bash
ros2 launch robot_control_nodes robot_control.launch.py \
  max_linear_vel:=0.8 \
  max_angular_vel:=1.5 \
  sync_queue_size:=20 \
  sync_slop:=0.2
```

Available parameters:
- `max_linear_vel`: Maximum linear velocity (default: 0.5 m/s)
- `max_angular_vel`: Maximum angular velocity (default: 1.0 rad/s)
- `sync_queue_size`: Synchronizer queue size (default: 10)
- `sync_slop`: Time synchronization window (default: 0.1 seconds)

## Usage

### Manual Launch (Alternative)

Launch each node individually in separate terminals:

```bash
# Terminal 1
ros2 run robot_control_nodes tracking_aruco

# Terminal 2
ros2 run robot_control_nodes lidar_stop

# Terminal 3
ros2 run robot_control_nodes mixer_node
```

### Custom Parameters

```bash
ros2 run robot_control_nodes mixer_node --ros-args \
  -p max_linear_vel:=0.8 \
  -p max_angular_vel:=1.5
```

## Dependencies

- rclpy
- geometry_msgs
- sensor_msgs
- cv_bridge
- python3-opencv
- python3-numpy

## Architecture

```
/camera/image_raw --> [tracking_aruco] --> /cmd_vel_angular -|
                                                               |--> [mixer_node] --> /cmd_vel --> Robot
/scan --> [lidar_stop] --> /cmd_vel_linear --------------------|
```

## Safety Features

- Velocity clamping to configured maximum values
- NaN/Inf filtering
- Smooth deceleration for obstacle avoidance
- Timestamped messages for synchronization
- Comprehensive error handling

