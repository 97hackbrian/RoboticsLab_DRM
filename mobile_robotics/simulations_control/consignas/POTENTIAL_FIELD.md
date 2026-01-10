# Potential Field Navigation - Mathematical Formulation

## Overview
Potential Field Navigation is a reactive navigation method that combines:
- **Attractive Force**: Pulls robot toward goal
- **Repulsive Force**: Pushes robot away from obstacles
- **Net Force**: Direction and magnitude guide robot motion

---

## 1. ATTRACTIVE FORCE (Goal Attraction)

### Mathematical Definition
The attractive force pulls the robot toward the goal position.

```
Goal Position: (goal_x, goal_y)
Robot Position: (x, y)

Vector to Goal:
  dx = goal_x - x
  dy = goal_y - y

Distance to Goal:
  d_goal = √(dx² + dy²)

Attractive Force Magnitude:
  F_att = k_att × d_goal

where k_att = attractive gain (default: 1.0)
```

### Force Components
```
Unit vector toward goal:
  û_goal = (dx / d_goal, dy / d_goal)

Attractive Force Vector:
  F_att_x = F_att × (dx / d_goal) = k_att × d_goal × (dx / d_goal) = k_att × dx
  F_att_y = F_att × (dy / d_goal) = k_att × d_goal × (dy / d_goal) = k_att × dy

Simplified:
  F_att_x = k_att × dx
  F_att_y = k_att × dy
```

### Code Implementation
```python
def calculate_attractive_force(self):
    dx = self.goal_x - self.robot_position.x
    dy = self.goal_y - self.robot_position.y
    distance = math.sqrt(dx**2 + dy**2)

    if distance < 0.01:  # Avoid division by zero
        return 0.0, 0.0

    # F_att = k_att * distance
    magnitude = self.k_attractive * distance

    # Direction: toward goal (normalized)
    fx = magnitude * (dx / distance)
    fy = magnitude * (dy / distance)

    return fx, fy
```

### Characteristics
- **Strength**: Proportional to distance (linear increase)
- **Direction**: Always toward goal
- **Range**: Infinite (but limited by repulsive forces)
- **Effect**: Never zero until goal is reached

---

## 2. REPULSIVE FORCE (Obstacle Avoidance)

### Laser Scan to Cartesian Conversion
The lidar returns distance measurements at different angles. We convert each reading to Cartesian coordinates in the robot frame.

#### Step 1: Extract Lidar Data
```
Lidar returns:
  - ranges[i]: distance to obstacle at angle i
  - angle_min: starting angle
  - angle_increment: angular resolution

For each reading i:
  angle_i = angle_min + i × angle_increment

  Example (typical lidar):
    angle_min = -π (-180°)
    angle_increment = π/360 (0.5° per reading)
```

#### Step 2: Convert to Robot Frame
Each lidar reading represents one point in space:

```
Obstacle Position in Robot Frame (Cartesian):
  obs_x = distance × cos(angle_i)
  obs_y = distance × sin(angle_i)

This gives the (x, y) position of each obstacle point
relative to the robot's center.

Distance from robot to this point:
  obs_dist = √(obs_x² + obs_y²) ≈ distance (same value)
```

### Mathematical Definition
The repulsive force uses an **inverse-square law** - stronger when closer:

```
Repulsive Force Magnitude (for one obstacle):

  F_rep = k_rep × (1/obs_dist - 1/ρ) / obs_dist²

  where:
    k_rep = repulsive gain (default: 0.3)
    ρ = repulsive_range (default: 0.5 m)
    obs_dist = distance to this obstacle point

This formula has two parts:
  1. (1/obs_dist - 1/ρ): Penalizes distance
     - When obs_dist < ρ: positive (repulsive)
     - When obs_dist ≥ ρ: negative or zero (no effect)

  2. 1/obs_dist²: Inverse square law
     - Force increases rapidly as robot approaches
     - At obs_dist = 0.1m: 100× stronger than at 1m
```

### Repulsive Force Components
```
Unit vector away from obstacle:
  û_obstacle = (-obs_x / obs_dist, -obs_y / obs_dist)
  Note: negative to point AWAY from obstacle

Repulsive Force Vector for this obstacle:
  F_rep_x = F_rep × (-obs_x / obs_dist)
  F_rep_y = F_rep × (-obs_y / obs_dist)

Total Repulsive Force (sum from ALL obstacles):
  F_rep_x_total = Σ F_rep_x (for all obstacles in range)
  F_rep_y_total = Σ F_rep_y (for all obstacles in range)
```

### Code Implementation
```python
def calculate_repulsive_force(self):
    if self.laser_data is None:
        return 0.0, 0.0

    fx_total = 0.0
    fy_total = 0.0
    angle_min = self.laser_data.angle_min
    angle_increment = self.laser_data.angle_increment

    # Iterate through all laser readings
    for i, distance in enumerate(self.laser_data.ranges):
        # Skip invalid readings
        if distance < self.laser_data.range_min or \
           distance > self.laser_data.range_max or \
           math.isinf(distance):
            continue

        # Only consider obstacles within repulsive range
        if distance > self.repulsive_range:
            continue

        # Step 1: Convert lidar reading to Cartesian coordinates
        angle = angle_min + i * angle_increment
        obs_x = distance * math.cos(angle)    # X component
        obs_y = distance * math.sin(angle)    # Y component

        # Step 2: Calculate distance
        obs_dist = math.sqrt(obs_x**2 + obs_y**2)

        if obs_dist < 0.01:  # Avoid division by zero
            continue

        # Step 3: Calculate repulsive force magnitude
        #   F_rep = k_rep × (1/obs_dist - 1/ρ) / obs_dist²
        magnitude = self.k_repulsive * \
                   (1.0 / obs_dist - 1.0 / self.repulsive_range) / \
                   (obs_dist ** 2)
        magnitude = max(0, magnitude)  # Only repulsive

        # Step 4: Calculate force components (away from obstacle)
        fx_total += magnitude * (-obs_x / obs_dist)
        fy_total += magnitude * (-obs_y / obs_dist)

    return fx_total, fy_total
```

### Force Magnitude Example
```
Given: k_rep = 0.3, ρ = 0.5m

At distance obs_dist = 0.2m:
  F_rep = 0.3 × (1/0.2 - 1/0.5) / 0.2²
        = 0.3 × (5 - 2) / 0.04
        = 0.3 × 3 / 0.04
        = 0.3 × 75
        = 22.5 (strong repulsion)

At distance obs_dist = 0.4m:
  F_rep = 0.3 × (1/0.4 - 1/0.5) / 0.4²
        = 0.3 × (2.5 - 2) / 0.16
        = 0.3 × 0.5 / 0.16
        = 0.3 × 3.125
        = 0.9375 (moderate repulsion)

At distance obs_dist = 0.5m (at range limit):
  F_rep = 0.3 × (1/0.5 - 1/0.5) / 0.5²
        = 0.3 × 0 / 0.25
        = 0 (no effect)
```

---

## 3. NET POTENTIAL FIELD (Combined Forces)

### Force Combination
```
Total Attractive Force:
  F_att_total = (F_att_x, F_att_y)

Total Repulsive Force:
  F_rep_total = (F_rep_x_total, F_rep_y_total)

Net Force (Superposition):
  F_net_x = F_att_x + F_rep_x_total
  F_net_y = F_att_y + F_rep_y_total

Net Force Magnitude:
  |F_net| = √(F_net_x² + F_net_y²)

Desired Direction:
  θ_desired = atan2(F_net_y, F_net_x)
```

### Code Implementation
```python
def control_loop(self):
    # Calculate forces
    fx_att, fy_att = self.calculate_attractive_force()
    fx_rep, fy_rep = self.calculate_repulsive_force()

    # Combine forces (superposition principle)
    fx_total = fx_att + fx_rep
    fy_total = fy_att + fy_rep

    # Calculate desired direction from combined forces
    force_magnitude = math.sqrt(fx_total**2 + fy_total**2)

    if force_magnitude < 0.01:  # No net force
        desired_angle = self.robot_yaw
    else:
        desired_angle = math.atan2(fy_total, fx_total)

    # Calculate steering angle error
    angle_error = self.normalize_angle(desired_angle - self.robot_yaw)
```

---

## 4. MOTION CONTROL

### Steering (Angular Velocity)
```
Angle Error:
  Δθ = θ_desired - θ_robot

Angular Velocity Command:
  ω = 2.0 × Δθ  (proportional gain = 2.0)
  ω = clamp(ω, -max_angular, max_angular)

This causes the robot to rotate toward the desired direction.
```

### Speed (Linear Velocity)
```
Speed Factor (obstacle proximity):
  If obstacle_dist < obstacle_threshold:
    speed_factor = obstacle_dist / obstacle_threshold
    speed_factor = max(0.1, speed_factor)  (minimum 10%)
  Else:
    speed_factor = 1.0

Linear Velocity Command:
  v = max_speed × speed_factor × speed_multiplier
  v = max(min_speed, v)  (ensure minimum forward motion)

This slows the robot near obstacles and speeds up in free space.
```

---

## 5. VECTOR VISUALIZATION

### Example: Robot with One Obstacle and Goal

```
Robot Position: (0, 0)
Goal Position: (2, 2)
Obstacle: (1, 0) at distance 1.0m

Attractive Force:
  dx = 2 - 0 = 2
  dy = 2 - 0 = 2
  d_goal = √(4 + 4) = 2.83m
  F_att = 1.0 × 2.83 = 2.83
  F_att_x = 2.83 × (2/2.83) = 2.0
  F_att_y = 2.83 × (2/2.83) = 2.0
  Direction: atan2(2.0, 2.0) = 45° (toward goal)

Repulsive Force (from obstacle at angle 0°):
  obs_x = 1.0 × cos(0°) = 1.0
  obs_y = 1.0 × sin(0°) = 0.0
  obs_dist = 1.0m
  F_rep = 0.3 × (1/1.0 - 1/0.5) / 1.0²
        = 0.3 × (1 - 2) / 1
        = 0.3 × (-1) = -0.3 (no effect, outside range)

Net Force:
  F_net_x = 2.0 + (-0.3) = 1.7
  F_net_y = 2.0 + 0.0 = 2.0
  Direction: atan2(2.0, 1.7) ≈ 49.6°
  (slightly deflected from goal direction to avoid obstacle)
```

---

## 6. KEY PARAMETERS & TUNING

### Attractive Force Gain (`k_attractive`)
```
Effect: How strongly the goal attracts the robot
- Higher value: Stronger pull toward goal, less obstacle avoidance
- Lower value: Weaker pull, robot avoids obstacles more

Default: 1.0
Typical range: 0.5 - 2.0
```

### Repulsive Force Gain (`k_repulsive`)
```
Effect: How strongly obstacles repel the robot
- Higher value: Stronger repulsion, robot stays further from obstacles
- Lower value: Weaker repulsion, robot may get closer

Default: 0.3
Typical range: 0.1 - 1.0
```

### Repulsive Range (`repulsive_range`)
```
Effect: How far obstacles influence the robot
- Larger range: Robot avoids obstacles earlier
- Smaller range: Robot waits until closer to obstacles

Default: 0.5m
Typical range: 0.3 - 1.0m
```

### Obstacle Threshold (`obstacle_threshold`)
```
Effect: Distance at which robot begins to decelerate
- Larger threshold: Robot slows down earlier
- Smaller threshold: Robot maintains speed longer

Default: 0.15m
Typical range: 0.1 - 0.5m
```

---

## 7. ADVANTAGES & LIMITATIONS

### Advantages
- ✓ Simple and computationally efficient
- ✓ Smooth, natural-looking paths
- ✓ Distributed obstacle avoidance
- ✓ Works in dynamic environments
- ✓ No path planning required

### Limitations
- ✗ **Local minima**: Robot can get stuck between obstacles
- ✗ **Oscillations**: Can oscillate near narrow passages
- ✗ **No global optimality**: Path may not be shortest
- ✗ **Parameter tuning**: Requires careful balance of gains

### Mitigation Strategies
- Use different k values for different scenarios
- Add tangential forces to escape local minima
- Implement stuck detection + escape behavior
- Use dynamic window approach as supplement

---

## 8. FORCE DIAGRAM EXAMPLE

```
Situation: Robot between goal and obstacle

        GOAL (2,2)
           ↗
          ⟲ F_att
         ╱
    ┌───────┐
    │ ROBOT │  ← F_rep
    └───────┘  ↙  (from obstacle)
        ↙
     F_net
     (combined)

The net force is a weighted sum of attraction and repulsion,
causing the robot to curve around the obstacle toward the goal.
```

---

## Summary

**Potential Field Navigation combines:**

1. **Attractive Force**: `F_att = k_att × (goal_position - robot_position)`
2. **Repulsive Force**: `F_rep = k_rep × Σ(1/d - 1/ρ)/d² × (-obstacle_direction)`
3. **Net Force**: `F_net = F_att + F_rep`
4. **Control**: Robot moves in direction of F_net with magnitude-based speed

This creates a smooth, reactive navigation behavior that adapts to the environment in real-time.
