"""
CARLA server connection and synchronous mode management.
"""
import carla
from .config import Config


def connect_to_carla(config: Config = None):
    """Connect to CARLA server."""
    if config is None:
        config = Config()
    
    client = carla.Client(config.carla_host, config.carla_port)
    client.set_timeout(config.carla_timeout)
    world = client.get_world()
    
    return client, world


def setup_synchronous_mode(world, client, config: Config = None):
    """Enable synchronous mode for deterministic sensor capture."""
    if config is None:
        config = Config()
    
    original_settings = world.get_settings()
    
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = config.fixed_delta_seconds
    world.apply_settings(settings)
    
    traffic_manager = client.get_trafficmanager(config.traffic_manager_port)
    traffic_manager.set_synchronous_mode(True)
    
    return original_settings, traffic_manager


def restore_async_mode(world, original_settings, traffic_manager):
    """Restore asynchronous mode after simulation."""
    world.apply_settings(original_settings)
    traffic_manager.set_synchronous_mode(False)
