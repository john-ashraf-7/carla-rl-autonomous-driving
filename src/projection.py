"""
LiDAR-to-camera projection for CARLA simulation.
Only saves projection images (no raw camera, LiDAR, or metadata).
"""
import numpy as np
import cv2
import os
from matplotlib import cm

# Viridis colormap for LiDAR intensity visualization
VIRIDIS = np.array(cm.get_cmap('viridis').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])


def project_lidar_to_camera(lidar_data, lidar, camera, camera_array, K, config):
    """
    Project 3D LiDAR points onto 2D camera image.
    
    Args:
        lidar_data: Raw LiDAR point cloud data
        lidar: LiDAR sensor actor
        camera: Camera sensor actor
        camera_array: Camera image as numpy array
        K: Camera intrinsic matrix
        config: Configuration object
        
    Returns:
        projected_image: Image with LiDAR points drawn
        num_points: Number of projected points
        weighted_density: Distance-weighted density for steering
    """
    image_w = config.camera_width
    image_h = config.camera_height
    
    # Parse point cloud (x, y, z, intensity)
    p_cloud = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')).reshape((-1, 4))
    intensity = p_cloud[:, 3]
    
    # Transform: LiDAR -> World -> Camera
    local_points = np.r_[p_cloud[:, :3].T, [np.ones(len(p_cloud))]]
    world_points = np.dot(lidar.get_transform().get_matrix(), local_points)
    sensor_points = np.dot(np.array(camera.get_transform().get_inverse_matrix()), world_points)
    
    # Convert to camera coordinates
    camera_coords = np.array([sensor_points[1], sensor_points[2] * -1, sensor_points[0]])
    
    # Project to 2D
    points_2d = np.dot(K, camera_coords)
    points_2d = np.array([points_2d[0] / points_2d[2], points_2d[1] / points_2d[2], points_2d[2]]).T
    
    # Filter points in image bounds
    mask = ((points_2d[:, 0] > 0) & (points_2d[:, 0] < image_w) & 
            (points_2d[:, 1] > 0) & (points_2d[:, 1] < image_h) & 
            (points_2d[:, 2] > 0))
    points_2d, intensity = points_2d[mask], intensity[mask]
    
    # Calculate distance-weighted density
    depths = points_2d[:, 2]
    weights = np.where(depths < config.close_distance_threshold, config.close_weight_multiplier, 1.0)
    weighted_density = np.sum(weights)
    
    # Colorize by intensity (Viridis colormap)
    intensity = np.clip(1.0 - (4 * intensity - 3), 0.0, 1.0)
    colors = np.array([
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 2]) * 255.0,
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 1]) * 255.0,
        np.interp(intensity, VID_RANGE, VIRIDIS[:, 0]) * 255.0
    ]).astype(np.uint8).T
    
    # Draw points on image
    im_array = camera_array.copy()
    u, v = points_2d[:, 0].astype(int), points_2d[:, 1].astype(int)
    dot_extent = config.dot_extent
    
    for i in range(len(points_2d)):
        v_min, v_max = max(0, v[i] - dot_extent), min(image_h, v[i] + dot_extent + 1)
        u_min, u_max = max(0, u[i] - dot_extent), min(image_w, u[i] + dot_extent + 1)
        im_array[v_min:v_max, u_min:u_max] = colors[i]
    
    return im_array, len(points_2d), weighted_density


def save_projection_image(cam_name, im_array, frame_num, output_dirs):
    """Save LiDAR projection image to appropriate camera folder."""
    filename = os.path.join(output_dirs[cam_name], f"frame_{frame_num:06d}.jpg")
    cv2.imwrite(filename, im_array)
    return filename


def image_to_array(image):
    """Convert CARLA image to numpy array."""
    return np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (image.height, image.width, 4)
    )[:, :, :3]
