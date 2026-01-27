"""
Trajectory-to-control converter for CARLA autonomous driving.
Translates target waypoint coordinates into throttle and steering commands.
"""
import numpy as np
import logging
import carla
from .config import Config

logger = logging.getLogger('TrajectoryControl')


class TrajectoryController:
    """
    Mathematical model that converts waypoint coordinates to vehicle control.
    
    Uses:
    - Pure pursuit algorithm for steering
    - Simple velocity control for throttle
    - Logging for debugging trajectory planning
    """
    
    def __init__(self, config: Config = None):
        """Initialize trajectory controller."""
        if config is None:
            config = Config()
        self.config = config
        self.last_vehicle_speed = 0.0
        
    def waypoint_to_control(self, waypoint_local, vehicle, analysis=None):
        """
        Convert target waypoint coordinates to throttle and steering.
        
        Mathematical model:
        1. Pure pursuit steering: calculates steering angle to reach waypoint
        2. Velocity control: adjusts throttle based on lateral deviation
        3. Acceleration/deceleration: smooth transitions
        
        Args:
            waypoint_local: (x, y, z) target coordinate in vehicle's local frame
            vehicle: CARLA vehicle actor
            analysis: dict with decision analysis data
            
        Returns:
            control: carla.VehicleControl with throttle and steer
            control_data: dict with detailed control information for logging
        """
        # Get vehicle state
        vehicle_velocity = vehicle.get_velocity()
        vehicle_speed = np.sqrt(vehicle_velocity.x**2 + vehicle_velocity.y**2 + vehicle_velocity.z**2)
        self.last_vehicle_speed = vehicle_speed
        
        # Extract waypoint coordinates
        lookahead_x = waypoint_local[0]
        lateral_y = waypoint_local[1]
        
        # ===== STEERING CALCULATION (Pure Pursuit) =====
        # Pure pursuit: steering angle = arctan(2 * L * sin(alpha) / lookahead_distance)
        # where L is vehicle wheelbase, alpha is angle to waypoint
        
        # Simplified: lateral deviation / lookahead distance
        # This gives us the angle (in radians) we need to steer
        if lookahead_x > 0.0001:  # Avoid division by zero
            # Cross-track error (lateral deviation)
            cross_track_error = lateral_y
            
            # Pure pursuit steering angle
            steering_angle = np.arctan(2.0 * cross_track_error / lookahead_x)
            
            # Convert to steering value [-1, 1]
            # Typical vehicle steering limits are ±25-30 degrees for full lock
            max_steering_angle = np.radians(self.config.max_steering_angle)
            steer = np.clip(steering_angle / max_steering_angle, -1.0, 1.0)
        else:
            steer = 0.0
        
        # ===== THROTTLE CALCULATION =====
        # Strategy:
        # 1. Base throttle from config
        # 2. Reduce throttle for sharp turns (high steering)
        # 3. Add brake if needed for tight curves
        
        # Base throttle
        throttle = self.config.throttle
        
        # Reduce throttle proportionally to steering angle (turn-slowing)
        steering_magnitude = abs(steer)
        throttle *= (1.0 - steering_magnitude * self.config.steering_throttle_reduction)
        
        # Brake if steering is very sharp (sharp turn)
        brake = 0.0
        if steering_magnitude > self.config.brake_steering_threshold:
            brake = steering_magnitude * self.config.brake_strength
            throttle = 0.0
        
        # Clamp values
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)
        steer = np.clip(steer, -1.0, 1.0)
        
        # Create CARLA control
        control = carla.VehicleControl()
        control.throttle = float(throttle)
        control.steer = float(steer)
        control.brake = float(brake)
        
        # Build detailed control data for logging
        control_data = {
            'cross_track_error': lateral_y,
            'lookahead_distance': lookahead_x,
            'steering_angle_rad': steering_angle if lookahead_x > 0.0001 else 0.0,
            'steering_angle_deg': np.degrees(steering_angle if lookahead_x > 0.0001 else 0.0),
            'steer_command': steer,
            'throttle_command': throttle,
            'brake_command': brake,
            'vehicle_speed_ms': vehicle_speed,
            'steering_magnitude': steering_magnitude
        }
        
        logger.debug(
            f"Control conversion | "
            f"Cross-track: {lateral_y:.3f}m | "
            f"Steering: {steer:.3f} | "
            f"Throttle: {throttle:.3f} | "
            f"Brake: {brake:.3f} | "
            f"Speed: {vehicle_speed:.2f}m/s"
        )
        
        return control, control_data
    
    def get_control_summary(self, control_data):
        """Generate human-readable summary of control state."""
        return (
            f"Steer:{control_data['steer_command']:+.2f} "
            f"Throttle:{control_data['throttle_command']:.2f} "
            f"Brake:{control_data['brake_command']:.2f} "
            f"Speed:{control_data['vehicle_speed_ms']:.1f}m/s"
        )
