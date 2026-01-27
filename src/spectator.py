"""
Spectator camera management for CARLA simulation.
"""
import carla


def update_spectator_view(world, vehicle, height=4.0, pitch=-15.0):
    """Position spectator camera behind and above vehicle."""
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(x=0, z=height),
        carla.Rotation(pitch=pitch, yaw=transform.rotation.yaw)
    ))
