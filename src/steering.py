"""
LiDAR density-based steering algorithm for CARLA simulation.
"""
import carla
from .config import Config


def lidar_density_steering(projection_counts, config: Config = None):
    """
    LiDAR density-based steering algorithm.
    
    Steers toward the direction with lower obstacle density:
    - Goes STRAIGHT when front has lowest density (clearest path ahead)
    - Steers proportionally based on density difference between sides
    
    Args:
        projection_counts: dict with 'front', 'left', 'right' weighted densities
        config: Configuration object
        
    Returns:
        carla.VehicleControl with throttle and proportional steering
    """
    if config is None:
        config = Config()
    
    front_count = projection_counts.get('front', 0)
    left_count = projection_counts.get('left', 0)
    right_count = projection_counts.get('right', 0)
    
    # If front has the lowest density (fewest obstacles), go straight
    if front_count < left_count and front_count < right_count:
        steer = 0.0
    else:
        # Calculate proportional steering based on density difference
        total_side = left_count + right_count
        
        if total_side > 0:
            # Positive = steer right, Negative = steer left
            density_diff = (left_count - right_count) / total_side
            steer = density_diff * config.steering_gain
        else:
            steer = 0.0
    
    # Clamp steering
    steer = max(-1.0, min(1.0, steer))
    
    control = carla.VehicleControl()
    control.throttle = config.throttle
    control.steer = steer
    control.brake = 0.0
    
    return control


def get_steering_direction(projection_counts):
    """Get human-readable steering direction for logging."""
    front = projection_counts.get('front', 0)
    left = projection_counts.get('left', 0)
    right = projection_counts.get('right', 0)
    
    if front < left and front < right:
        return "STRAIGHT (front clear)"
    elif left > right:
        return "RIGHT"
    elif right > left:
        return "LEFT"
    else:
        return "STRAIGHT"
