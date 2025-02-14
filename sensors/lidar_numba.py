"""
A replacement for lidar.py as
from lidar_numba import simulate_2d_lidar_scan

leverage numba for >= 10x faster ray casting
"""

import numpy as np
from typing import List, Tuple, Optional
from numba import jit, prange


@jit(nopython=True, parallel=True, fastmath=True)
def _fast_ray_march(
    grid_map: np.ndarray,
    start_x: float,
    start_y: float,
    directions_x: np.ndarray,
    directions_y: np.ndarray,
    max_range: float,
    step_size: float,
    width: int,
    height: int,
) -> np.ndarray:
    """
    Optimized ray marching using Numba for parallel processing.
    Returns the distance to the first obstacle for each ray.
    """
    num_rays = len(directions_x)
    hit_distances = np.full(num_rays, max_range)

    # Process each ray in parallel
    for ray_idx in prange(num_rays):
        dx = directions_x[ray_idx]
        dy = directions_y[ray_idx]

        # Current position
        x = start_x
        y = start_y
        distance = 0.0

        # Pre-compute step components
        step_x = dx * step_size
        step_y = dy * step_size

        while distance < max_range:
            # Update position
            x += step_x
            y += step_y
            distance += step_size

            # Convert to grid coordinates
            grid_x = int(x)
            grid_y = int(y)

            # Check bounds
            if not (0 <= grid_x < width and 0 <= grid_y < height):
                hit_distances[ray_idx] = distance
                break

            # Check for obstacle
            if grid_map[grid_y, grid_x] == 0:
                hit_distances[ray_idx] = distance
                break

    return hit_distances


@jit(nopython=True)
def _distances_to_points(
    start_x: float,
    start_y: float,
    directions_x: np.ndarray,
    directions_y: np.ndarray,
    distances: np.ndarray,
) -> np.ndarray:
    """Convert ray distances to endpoint coordinates."""
    num_rays = len(distances)
    points = np.empty((num_rays, 2))

    for i in range(num_rays):
        points[i, 0] = start_x + directions_x[i] * distances[i]
        points[i, 1] = start_y + directions_y[i] * distances[i]

    return points


def simulate_2d_lidar_scan(
    grid_map: np.ndarray,
    position: Tuple[float, float],
    angle_min: float = 0,
    angle_max: float = 2 * np.pi,
    angular_resolution: float = np.pi / 180,  # 1 degree
    max_range: float = 100.0,
    step_size: float = 0.5,  # Smaller step size for better accuracy
) -> List[Tuple[float, float]]:
    """
    Ultra-fast 2D LiDAR simulation using Numba-accelerated parallel ray marching.

    Args:
        grid_map (np.ndarray): Binary grid map (0=obstacle, 1=free)
        position (tuple): (x, y) position of the LiDAR
        angle_min (float): Start angle in radians
        angle_max (float): End angle in radians
        angular_resolution (float): Angle between rays in radians
        max_range (float): Maximum sensing range
        step_size (float): Ray marching step size (smaller = more accurate)

    Returns:
        List[Tuple[float, float]]: Detected obstacle points
    """
    height, width = grid_map.shape
    start_x, start_y = position

    # Input validation
    if not (0 <= start_x < width and 0 <= start_y < height):
        return []

    # Ensure grid_map is in the correct type for Numba
    grid_map = grid_map.astype(np.int_)

    # Generate angles
    angles = np.arange(angle_min, angle_max + angular_resolution / 2, angular_resolution)

    # Pre-compute direction vectors (more efficient than storing angles)
    directions_x = np.cos(angles)
    directions_y = np.sin(angles)

    # Perform parallel ray marching
    hit_distances = _fast_ray_march(
        grid_map, start_x, start_y, directions_x, directions_y, max_range, step_size, width, height
    )

    # Convert distances to points
    hit_points = _distances_to_points(start_x, start_y, directions_x, directions_y, hit_distances)

    filtered_points = [(float(x), float(y)) for (x, y), d in zip(hit_points, hit_distances) if d < (max_range - 1)]

    # Convert to list of tuples (as per original interface)
    return filtered_points


def create_obstacle_binary_map(
    width: int, height: int, centers: List[Tuple[int, int]], radii: List[int]
) -> np.ndarray:
    """
    Create a grid map with specified obstacles. The grid map is represented as a binary matrix where 1 represents
    free space and 0 represents an obstacle.

    Args:
        width (int): Width of the grid map.
        height (int): Height of the grid map.
        centers (List[Tuple[int, int]]): List of obstacle centers as (x, y) tuples.
        radii (List[int]): List of obstacle radii.

    """

    def draw_circle(grid: np.ndarray, centers: List[Tuple[int, int]], radii: List[int]) -> np.ndarray:
        y, x = np.ogrid[: grid.shape[0], : grid.shape[1]]
        # consider vectorization when the list of centers and radii is large
        for center, radius in zip(centers, radii):
            mask = (x - center[0]) ** 2 + (y - center[1]) ** 2 <= radius**2
            grid[mask] = 0
        return grid

    grid = np.ones((height, width), dtype=int)
    grid = draw_circle(grid, centers, radii)

    grid[:3, :] = 0
    grid[-3:, :] = 0
    grid[:, :3] = 0
    grid[:, -3:] = 0
    return grid
