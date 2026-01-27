# Quick Reference Guide - Waypoint Navigation System

## What Changed?

### Old Flow
```
LiDAR Density → Direct Steering Logic → throttle + steer → Vehicle
```

### New Flow
```
LiDAR Density → Waypoint Coordinates → Pure Pursuit Steering → throttle + steer + brake → Vehicle
```

---

## Running the Simulation

```python
# In main.ipynb - no changes needed to imports, all integrated

trajectory_controller = TrajectoryController(config)

# Each simulation tick:
waypoint_local, direction, analysis = lidar_density_to_waypoint(
    projection_counts, vehicle, config
)
control, control_data = trajectory_controller.waypoint_to_control(
    waypoint_local, vehicle, analysis
)
vehicle.apply_control(control)
```

---

## Key Concepts

### Waypoint Coordinates
- **X (forward)**: 0-10m ahead of vehicle
- **Y (right)**: -5m (left) to +5m (right)
- **Z (up)**: always 0 (ground level)

Example: `waypoint = [10, 2.5, 0]` = 10m ahead, 2.5m to the right

### Steering Calculation
```
steering_angle = arctan(2 * lateral_deviation / lookahead_distance)
```
- Larger lateral deviation → sharper steering
- Larger lookahead → gentler steering
- **Pure pursuit** = proven algorithm for vehicle path tracking

### Throttle Strategy
```
throttle = 0.3 * (1 - |steer| * 0.5)
```
- Reduced during turns (prevents sliding)
- Full power going straight
- Smooth gradual changes

### Brake Activation
```
if |steer| > 0.7:  # Very sharp turn
    brake = |steer| * 0.3
```
- Only for emergency sharp turns
- Intensity scales with sharpness

---

## Configuration Tuning

### To Make Vehicle Turn Sharper
```python
config.max_lateral_offset = 7.0  # Increase from 5.0
config.lookahead_distance = 8.0  # Decrease from 10.0
```

### To Make Vehicle Smoother
```python
config.max_lateral_offset = 3.0  # Decrease from 5.0
config.lookahead_distance = 15.0  # Increase from 10.0
```

### To Go Faster Through Turns
```python
config.steering_throttle_reduction = 0.2  # Reduce from 0.5
config.brake_steering_threshold = 0.9  # Increase from 0.7
```

### To Go Slower (Safer)
```python
config.throttle = 0.2  # Decrease from 0.3
config.steering_throttle_reduction = 0.8  # Increase from 0.5
```

---

## Debugging

### Check Waypoint Direction
```
"Front clear" → STRAIGHT, minimal steering
"Left obstacle" → RIGHT, positive steering
"Right obstacle" → LEFT, negative steering
```

### Monitor Steering Angle
```
0° = straight
±15° = moderate turn
±25° = sharp turn (brake activated)
```

### Track Vehicle Speed
```
Full throttle (0.3) on straight
Reduced to 0.15-0.225 during turns (due to steering reduction)
Further reduced if brake applied (emergency)
```

### Parse Log Output
```
[5s] Speed: 8.3 km/h | Densities L:45 F:23 R:67 | 
Waypoint: (10.00m, 2.35m) | Steer:+0.28 Throttle:0.25 Brake:0.00

Explanation:
- 5 seconds into simulation
- Going 8.3 km/h (about 2.3 m/s)
- LiDAR shows obstacles: left=45, front=23, right=67
- Generated waypoint: 10m ahead, 2.35m to the right
- Control: 0.28 right steer, 0.25 throttle (reduced from 0.3), no braking
```

---

## Common Issues & Solutions

### Vehicle Oscillates Side-to-Side
- **Cause**: Lookahead distance too short
- **Fix**: Increase `config.lookahead_distance` to 12-15m

### Vehicle Never Turns Sharp Enough
- **Cause**: Lateral offset or steering gain too low
- **Fix**: Increase `config.max_lateral_offset` or decrease `config.max_steering_angle`

### Vehicle Overshoots Waypoints
- **Cause**: Pure pursuit gain too aggressive
- **Fix**: Increase `config.lookahead_distance`

### Vehicle Moves Too Fast on Turns
- **Cause**: Steering-throttle reduction too weak
- **Fix**: Increase `config.steering_throttle_reduction` (0.5 → 0.7)

### Logs Are Too Verbose
- **Fix**: Change logging level in main.ipynb
```python
logging.basicConfig(level=logging.INFO)  # Instead of DEBUG
```

---

## Mathematical Properties

| Parameter | Equation | Effect |
|-----------|----------|--------|
| Steering Angle | `arctan(2y/x)` | Higher y → sharper turn |
| Cross-Track Error | `y` | Perpendicular distance to ideal path |
| Throttle Reduction | `1 - \|s\| * 0.5` | Smoother at low speeds |
| Brake Threshold | 0.7 | Activates when steering > ±70% |

---

## Files Reference

| File | Purpose |
|------|---------|
| `src/waypoint_planning.py` | LiDAR density → waypoint generation |
| `src/trajectory_control.py` | Waypoint → throttle/steer/brake conversion |
| `src/config.py` | All tunable parameters |
| `main.ipynb` | Main simulation loop using waypoint system |
| `carla_navigation.log` | Debug log file (created during run) |

---

## Next Steps

1. **Run simulation** with default parameters
2. **Analyze log file** for any warnings or errors
3. **Adjust tuning** based on vehicle behavior
4. **Enable visualization** with matplotlib to plot waypoints
5. **Validate control** by comparing commanded vs actual steering

