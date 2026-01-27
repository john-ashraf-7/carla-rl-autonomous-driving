"""
Waypoint-based planning for CARLA autonomous driving.
Converts LiDAR density analysis into target waypoint coordinates.
"""
import numpy as np
import logging
from .config import Config

logger = logging.getLogger('WaypointPlanning')


def lidar_density_to_waypoint(projection_counts, vehicle, config: Config = None):
    """
    Convert LiDAR density analysis to target waypoint coordinates.
    
    Instead of directly outputting throttle/steer, this function:
    1. Analyzes lidar density to determine preferred direction
    2. Generates a waypoint coordinate ahead of the vehicle
    3. Returns coordinate in vehicle's local coordinate system
    
    Args:
        projection_counts: dict with 'front', 'left', 'right' weighted densities
        vehicle: CARLA vehicle actor
        config: Configuration object
        
    Returns:
        waypoint: (x, y, z) target coordinate in vehicle's local frame
        direction: string describing chosen direction
        analysis: dict with detailed decision info for logging
    """
    if config is None:
        config = Config()
    
    front_count = projection_counts.get('front', 0)
    left_count = projection_counts.get('left', 0)
    right_count = projection_counts.get('right', 0)
    
    # Get vehicle transform for coordinate conversion
    vehicle_transform = vehicle.get_transform()
    vehicle_location = vehicle_transform.location
    vehicle_rotation = vehicle_transform.rotation
    
    # Determine preferred direction based on density
    # Lower density = clearer path = preferred direction
    if front_count < left_count and front_count < right_count:
        direction = "STRAIGHT"
        lateral_offset = 0.0
    else:
        # Choose between left and right based on which has lower density
        if left_count > right_count:
            direction = "RIGHT"
            # Steer right: positive lateral offset
            density_diff = (left_count - right_count) / max(left_count + right_count, 1.0)
            lateral_offset = density_diff * config.max_lateral_offset
        else:
            direction = "LEFT"
            # Steer left: negative lateral offset
            density_diff = (right_count - left_count) / max(left_count + right_count, 1.0)
            lateral_offset = -density_diff * config.max_lateral_offset
    
    # Clamp lateral offset
    lateral_offset = max(-config.max_lateral_offset, min(config.max_lateral_offset, lateral_offset))
    
    # Generate waypoint in vehicle's local coordinate system
    # Local frame: X=forward, Y=right, Z=up
    lookahead_distance = config.lookahead_distance
    waypoint_local = np.array([
        lookahead_distance,      # Forward
        lateral_offset,          # Right (positive = right, negative = left)
        0.0                      # Level
    ])
    
    analysis = {
        'front_density': front_count,
        'left_density': left_count,
        'right_density': right_count,
        'direction': direction,
        'lateral_offset': lateral_offset,
        'lookahead_distance': lookahead_distance,
        'waypoint_local': waypoint_local
    }
    
    logger.debug(
        f"Waypoint decision | Direction: {direction} | "
        f"Densities [L:{left_count:.1f} F:{front_count:.1f} R:{right_count:.1f}] | "
        f"Lateral offset: {lateral_offset:.3f}m"
    )
    
    return waypoint_local, direction, analysis


def get_waypoint_direction_string(direction, lateral_offset):
    """Get human-readable direction string for logging."""
    if direction == "STRAIGHT":
        return "STRAIGHT (front clear)"
    elif direction == "RIGHT":
        return f"RIGHT ({lateral_offset:.2f}m)"
    elif direction == "LEFT":
        return f"LEFT ({lateral_offset:.2f}m)"
    else:
        return "UNKNOWN"
