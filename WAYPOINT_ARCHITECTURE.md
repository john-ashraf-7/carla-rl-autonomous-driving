# Waypoint-Based Navigation System - Implementation Summary

## Overview
Refactored the autonomous driving system from direct throttle/steer output to a **coordinate-based waypoint planning** approach with a mathematical model that translates waypoints to vehicle controls.

---

## Architecture

```
LiDAR Projection (density analysis)
         ↓
Waypoint Planning (coordinate generation)
         ↓
Trajectory Controller (coordinate → control conversion)
         ↓
Vehicle Control (throttle/steer/brake)
```

---

## New Modules

### 1. **waypoint_planning.py** - Coordinate Generation
**Function:** `lidar_density_to_waypoint(projection_counts, vehicle, config)`

- Analyzes LiDAR density (front, left, right) to determine preferred direction
- Generates target waypoint coordinates in vehicle's local coordinate frame
- **Coordinate System:**
  - X-axis: Forward (0-10m lookahead)
  - Y-axis: Right (±5m lateral offset)
  - Z-axis: Level (always 0)

**Logic:**
- If front has lowest density → STRAIGHT (lateral_offset = 0)
- Else → choose LEFT or RIGHT based on which side has lower density
- Lateral offset scales with density difference (0 to ±max_lateral_offset)

**Returns:**
- `waypoint_local`: (x, y, z) coordinates in vehicle frame
- `direction`: "STRAIGHT", "LEFT", or "RIGHT"
- `analysis`: dict with detailed decision metrics

---

### 2. **trajectory_control.py** - Waypoint to Control Conversion
**Class:** `TrajectoryController`

**Method:** `waypoint_to_control(waypoint_local, vehicle, analysis)`

#### Mathematical Model

##### Pure Pursuit Steering
```
steering_angle = arctan(2 * lateral_deviation / lookahead_distance)
steer_value = steering_angle / max_steering_angle (normalized to [-1, 1])
```
- Cross-track error: `lateral_y` (how far sideways from ideal path)
- Steering proportional to deviation and inversely proportional to distance
- **Max steering angle:** 25° (configurable)

##### Throttle Control
```
base_throttle = config.throttle (0.3)
throttle_reduced = throttle * (1 - |steer| * steering_throttle_reduction)
throttle_final = max(0, throttle_reduced)
```
- **Steering-based throttle reduction:** Reduces throttle during sharp turns
- **Reduction factor:** 0.5 (50% reduction at maximum steer)

##### Brake Control
```
if |steer| > brake_steering_threshold (0.7):
    brake = |steer| * brake_strength (0.3)
else:
    brake = 0
```
- Activates for sharp turns (steer > ±0.7)
- Intensity scales with steering magnitude

**Returns:**
- `control`: `carla.VehicleControl` object (throttle, steer, brake)
- `control_data`: dict with detailed metrics (angles, speeds, magnitudes)

---

## Configuration Parameters

Added to `Config` class:

```python
# Waypoint Planning
lookahead_distance: float = 10.0      # How far ahead to plan (meters)
max_lateral_offset: float = 5.0       # Max left/right deviation (meters)

# Trajectory Control
max_steering_angle: float = 25.0      # Max steering lock (degrees)
steering_throttle_reduction: float = 0.5  # Throttle reduction during turns
brake_steering_threshold: float = 0.7 # Steer magnitude to trigger brake
brake_strength: float = 0.3           # Brake intensity (0-1)
```

---

## Logging System

### Three Levels of Logging

1. **DEBUG**: Detailed per-tick information
   - Waypoint decisions
   - Cross-track errors
   - Control calculations
   - Vehicle speed and steering angles

2. **INFO**: 5-second summary snapshots
   - Current simulation time
   - Vehicle speed (km/h)
   - Density readings
   - Generated waypoint coordinates
   - Applied throttle/steer/brake values

3. **WARNING**: Important events
   - Simulation interruptions
   - Sensor timeouts
   - Safety thresholds exceeded

### Log Output
- **Console**: Real-time feedback
- **File**: `carla_navigation.log` (persistent record)

### Sample Log Entry
```
[Navigation] INFO: [5s] Speed: 8.3 km/h | Densities L:45 F:23 R:67 | 
Waypoint: (10.00m, 2.35m) | Steer:+0.28 Throttle:0.25 Brake:0.00 Speed:8.3m/s
```

---

## Data Flow

```
LiDAR Points (3D) 
  → Projected to Cameras (2D images)
  → Density analyzed per camera (front/left/right)
  → Converted to Waypoint Coordinates (10m ahead, ±5m lateral)
  → Translated to Steering Angle (pure pursuit algorithm)
  → Converted to Control Values (throttle, steer, brake)
  → Applied to Vehicle (CARLA physics)
```

---

## Key Features

### 1. **Explainable Decisions**
- Waypoint coordinates show exactly what direction algorithm chose
- Control data shows steering angle, cross-track error, etc.

### 2. **Safety-Aware**
- Reduces throttle during turns (prevents skidding)
- Applies brakes for sharp turns (prevents high-speed maneuvers)
- Clamped values prevent extreme commands

### 3. **Debuggable**
- Detailed logging at every stage
- Human-readable direction strings
- Metrics for trajectory analysis
- File-based log for post-simulation analysis

### 4. **Tunable**
- Easy to adjust lookahead distance, lateral limits
- Steering-to-throttle relationship configurable
- Brake thresholds adjustable

---

## Usage in main.ipynb

```python
# Initialize controller
trajectory_controller = TrajectoryController(config)

# Each tick:
waypoint_local, direction, analysis = lidar_density_to_waypoint(
    projection_counts, vehicle, config
)
control, control_data = trajectory_controller.waypoint_to_control(
    waypoint_local, vehicle, analysis
)
vehicle.apply_control(control)
```

---

## Debug Workflow

1. **Check waypoint generation**: Look for unexpected `direction` or `lateral_offset` values
2. **Verify steering math**: Compare `steering_angle_rad` with expected arctan value
3. **Monitor throttle reduction**: `steering_magnitude * 0.5` should match throttle decrease
4. **Track vehicle response**: Compare commanded steer with actual vehicle heading
5. **Analyze logs**: Use `carla_navigation.log` to correlate decisions across ticks

---

## Example Scenarios

### Obstacle on Left (Steer Right)
```
Input:  L_density=80, F_density=20, R_density=40
Output: direction="RIGHT", lateral_offset=+2.5m
        steering_angle=+14°, steer=+0.56
Result: Vehicle steers right, throttle reduced to 0.22
```

### Clear Path Ahead (Go Straight)
```
Input:  L_density=70, F_density=10, R_density=75
Output: direction="STRAIGHT", lateral_offset=0m
        steering_angle=0°, steer=0.0
Result: Vehicle goes straight, full throttle 0.3
```

### Tight Right Turn (Apply Brake)
```
Input:  L_density=90, F_density=50, R_density=30
Output: direction="RIGHT", lateral_offset=+5.0m
        steering_angle=+25°, steer=1.0 (full lock)
        brake=0.30
Result: Vehicle brakes to 0.15, full right steer
```

---

## Files Modified

1. **src/waypoint_planning.py** - NEW
2. **src/trajectory_control.py** - NEW
3. **src/config.py** - Updated with new parameters
4. **src/__init__.py** - Updated to export new modules
5. **main.ipynb** - Refactored simulation loop
   - Added logging setup
   - Replaced `lidar_density_steering()` with waypoint pipeline
   - Added waypoint and control data tracking
   - Enhanced status output with waypoint info
