"""
Vehicle spawning for CARLA simulation.
"""
import carla
from .config import Config


def spawn_vehicle(world, config: Config = None):
    """Spawn vehicle at configured location."""
    if config is None:
        config = Config()
    
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter(config.vehicle_blueprint)[0]
    
    spawn_point = carla.Transform(
        carla.Location(x=config.spawn_x, y=config.spawn_y, z=config.spawn_z),
        carla.Rotation(pitch=0, yaw=config.spawn_yaw, roll=0)
    )
    
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    world.tick()
    
    return vehicle
