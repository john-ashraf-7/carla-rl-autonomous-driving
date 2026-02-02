"""
Configuration settings for CARLA autonomous driving simulation.
"""
from dataclasses import dataclass
from datetime import datetime
import os


@dataclass
class Config:
    """Central configuration for the CARLA simulation."""
    
    # CARLA Connection
    carla_host: str = 'localhost'
    carla_port: int = 2000
    carla_timeout: float = 20.0
    traffic_manager_port: int = 8000
    
    # Simulation Settings
    fps: int = 20  # fixed_delta_seconds = 1/fps
    simulation_duration: float = 30.0  # seconds
    capture_interval: float = 0.5  # seconds
    
    # Vehicle Spawn Location
    spawn_x: float = 35.7
    spawn_y: float = 24.8
    spawn_z: float = 0.6
    spawn_yaw: float = 0.2  # degrees
    vehicle_blueprint: str = 'vehicle.tesla.model3'
    
    # Camera Configuration
    camera_width: int = 800
    camera_height: int = 600
    camera_fov: int = 90
    
    # LiDAR Configuration
    lidar_channels: int = 16
    lidar_points_per_second: int = 300000
    lidar_rotation_frequency: int = 20
    lidar_range: int = 120  # meters
    lidar_upper_fov: int = 10
    lidar_lower_fov: int = -30
    
    # LiDAR Obstacle Detection Parameters
    steering_gain: float = 2.5  # Higher = more aggressive steering response
    close_distance_threshold: float = 80.0  # meters - detect obstacles earlier
    close_weight_multiplier: float = 6.0  # Stronger reaction to close obstacles
    
    # Waypoint Planning Parameters
    lookahead_distance: float = 10.0  # meters - how far ahead to plan
    max_lateral_offset: float = 5.0  # meters - max left/right deviation
    
    # PID Controller Parameters
    target_speed_kmh: float = 20.0  # Target cruise speed in km/h
    
    # Lateral PID (steering control)
    lateral_K_P: float = 1.95   # Proportional - main steering response
    lateral_K_I: float = 0.05  # Integral - corrects steady-state error
    lateral_K_D: float = 0.2   # Derivative - dampens oscillation
    
    # Longitudinal PID (throttle/brake control)
    longitudinal_K_P: float = 1.0   # Proportional - main speed response
    longitudinal_K_I: float = 0.05  # Integral - maintains target speed
    longitudinal_K_D: float = 0.0   # Derivative - usually 0 for speed control
    
    # Control Limits
    max_throttle: float = 0.75  # Maximum throttle [0-1]
    max_brake: float = 0.3      # Maximum brake [0-1]
    max_steering: float = 0.8   # Maximum steering magnitude [0-1]
    
    # Visualization
    dot_extent: int = 2  # LiDAR projection dot size
    
    # Output Configuration
    output_base_dir: str = 'output'
    
    @property
    def fixed_delta_seconds(self) -> float:
        return 1.0 / self.fps
    
    @property
    def capture_interval_ticks(self) -> int:
        return int(self.capture_interval / self.fixed_delta_seconds)
    
    @property
    def total_ticks(self) -> int:
        return int(self.simulation_duration / self.fixed_delta_seconds)
    
    @property
    def pid_dt(self) -> float:
        """Time step for PID controller (matches simulation tick rate)."""
        return 1.0 / self.fps
    
    @property
    def args_lateral(self) -> dict:
        """Lateral PID parameters dict for VehiclePIDController."""
        return {'K_P': self.lateral_K_P, 'K_I': self.lateral_K_I, 
                'K_D': self.lateral_K_D, 'dt': self.pid_dt}
    
    @property
    def args_longitudinal(self) -> dict:
        """Longitudinal PID parameters dict for VehiclePIDController."""
        return {'K_P': self.longitudinal_K_P, 'K_I': self.longitudinal_K_I,
                'K_D': self.longitudinal_K_D, 'dt': self.pid_dt}
    
    def get_output_dir(self) -> str:
        """Get timestamped output directory path."""
        timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        return os.path.join(self.output_base_dir, timestamp)
    
    def create_output_dirs(self, output_dir: str) -> dict:
        """Create output directory structure for LiDAR projections only."""
        dirs = {
            'base': output_dir,
            'projection': os.path.join(output_dir, 'lidar_projection'),
            'front': os.path.join(output_dir, 'lidar_projection', 'camera_front'),
            'left': os.path.join(output_dir, 'lidar_projection', 'camera_left'),
            'right': os.path.join(output_dir, 'lidar_projection', 'camera_right'),
        }
        for path in dirs.values():
            os.makedirs(path, exist_ok=True)
        return dirs
