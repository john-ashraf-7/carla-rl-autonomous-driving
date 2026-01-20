# CARLA simulation of a reinforcement learning self-driving model

The goal of this simulation is to test a self-driving reinforcement learning model using input from 3 cameras and a LiDAR attached to the vehicle.

## Simulation Features

- **Synchronous simulation** - Deterministic sensor data capture
- **Multi-camera setup** - Front, left, and right cameras (1920x1080, 90° FOV)
- **LiDAR sensor** - 64-channel with 100m range
- **Sensor fusion** - LiDAR-to-camera projection using intrinsic matrices
- **AI navigation** - Obstacle detection and avoidance using projected LiDAR data
- **Clean environment** - All map objects removed except roads

## Requirements

```bash
pip install -r requirements.txt
```

## Pipeline Overview

### 1. Connection & Configuration
- Connect to CARLA server
- Enable synchronous mode (20 FPS, 50ms ticks)
- Configure traffic manager

### 2. Environment Setup
- [OPTIONAL] Clear all map layers (buildings, props, foliage)
- [OPTIONAL] Keep only road surfaces for clean environment

### 3. Vehicle & Sensors
- Spawn Tesla Model 3
- Attach 3 RGB cameras (front/left/right)
- Attach 64-channel LiDAR on roof
- Compute camera intrinsic matrix for projection

### 4. Data Recording
- Synchronous queues ensure temporal alignment
- LiDAR points projected onto camera images
- Intensity-based colorization (Viridis colormap)
- Output structure:
  ```
  carla_output/
  ├── camera_front/       # Raw camera images
  ├── camera_left/
  ├── camera_right/
  ├── lidar/              # Point clouds (.npy)
  ├── lidar_projection/   # LiDAR projected on images
  └── fusion/             # JSON metadata linking sensors
  ```

### 5. AI Controller
- Analyzes front camera LiDAR projection
- Divides view into zones (left/center/right)
- Detects obstacles by point density
- Computes steering/throttle based on clearance
- Smooth control with steering history

### 6. Simulation Loop
- Advance simulation tick-by-tick
- Wait for synchronized sensor data
- Process and save data at intervals
- Apply AI control commands
- Update spectator camera view

### 7. Cleanup
- Stop sensor listeners
- Destroy all actors
- Restore asynchronous mode

## Usage

1. Start CARLA simulator:
   ```bash
   ./CarlaUE4.sh
   ```

2. Run notebook cells sequentially

3. Monitor simulation in CARLA window

4. Data saved to `carla_output/` directory

## Configuration

Key parameters in notebook:
- `simulation_duration` - Run time in seconds (default: 30s)
- `capture_interval_seconds` - Data save frequency (default: 0.5s)
- `obstacle_threshold` - AI obstacle detection sensitivity (default: 0.15)
- `ai_default_throttle` - Forward speed (default: 0.5)

## Notes

- Synchronous mode ensures frame alignment across sensors
- LiDAR projection uses camera intrinsics (K matrix)
- AI uses simple zone-based obstacle avoidance
- Spectator follows vehicle in third-person view
