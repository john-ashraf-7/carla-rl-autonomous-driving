"""
Camera and LiDAR sensor setup for CARLA simulation.
"""
import carla
import numpy as np
import queue
from .config import Config


# Camera positions relative to vehicle
CAMERA_TRANSFORMS = {
    'front': carla.Transform(carla.Location(x=2.5, z=1.0), carla.Rotation(pitch=0, yaw=0)),
    'left': carla.Transform(carla.Location(x=0.0, y=-1.0, z=1.0), carla.Rotation(pitch=0, yaw=-90)),
    'right': carla.Transform(carla.Location(x=0.0, y=1.0, z=1.0), carla.Rotation(pitch=0, yaw=90)),
}

# LiDAR position relative to vehicle
LIDAR_TRANSFORM = carla.Transform(carla.Location(x=0.0, z=2.5), carla.Rotation(pitch=0, yaw=0))


def spawn_cameras(world, vehicle, config: Config = None):
    """Spawn and attach RGB cameras to vehicle."""
    if config is None:
        config = Config()
    
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(config.camera_width))
    camera_bp.set_attribute('image_size_y', str(config.camera_height))
    camera_bp.set_attribute('fov', str(config.camera_fov))
    
    cameras = {}
    for cam_name, transform in CAMERA_TRANSFORMS.items():
        cameras[cam_name] = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
    world.tick()
    
    # Compute camera intrinsic matrix K
    focal = config.camera_width / (2.0 * np.tan(config.camera_fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = config.camera_width / 2.0
    K[1, 2] = config.camera_height / 2.0
    
    return cameras, K


def spawn_lidar(world, vehicle, config: Config = None):
    """Spawn and attach LiDAR sensor to vehicle."""
    if config is None:
        config = Config()
    
    blueprint_library = world.get_blueprint_library()
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels', str(config.lidar_channels))
    lidar_bp.set_attribute('points_per_second', str(config.lidar_points_per_second))
    lidar_bp.set_attribute('rotation_frequency', str(config.lidar_rotation_frequency))
    lidar_bp.set_attribute('range', str(config.lidar_range))
    lidar_bp.set_attribute('upper_fov', str(config.lidar_upper_fov))
    lidar_bp.set_attribute('lower_fov', str(config.lidar_lower_fov))
    
    lidar = world.spawn_actor(lidar_bp, LIDAR_TRANSFORM, attach_to=vehicle)
    world.tick()
    
    return lidar


def create_sensor_queues(cameras):
    """Create synchronized queues for sensor data."""
    camera_queues = {cam_name: queue.Queue() for cam_name in cameras.keys()}
    lidar_queue = queue.Queue()
    return camera_queues, lidar_queue


def create_camera_callback(camera_queues, cam_name):
    """Create callback function for camera data."""
    def callback(image):
        camera_queues[cam_name].put(image)
    return callback


def create_lidar_callback(lidar_queue):
    """Create callback function for LiDAR data."""
    def callback(point_cloud):
        lidar_queue.put(point_cloud)
    return callback
