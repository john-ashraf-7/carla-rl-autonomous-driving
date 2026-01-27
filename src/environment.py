"""
CARLA environment management - map layer loading/unloading.
"""
import carla


MAP_LAYERS = [
    carla.MapLayer.Buildings,
    carla.MapLayer.Decals,
    carla.MapLayer.Foliage,
    carla.MapLayer.ParkedVehicles,
    carla.MapLayer.Particles,
    carla.MapLayer.Props,
    carla.MapLayer.Walls,
]


def clear_environment(world):
    """Remove all static map objects (keep only roads)."""
    for layer in MAP_LAYERS:
        world.unload_map_layer(layer)
    world.tick()


def reload_environment(world):
    """Reload all static map objects."""
    for layer in MAP_LAYERS:
        world.load_map_layer(layer)
    world.tick()
