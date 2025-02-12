import numpy as np
from typing import List, Tuple, Optional


def simulate_2d_lidar_scan(
    grid_map: np.ndarray,
    position: Tuple[float, float],
    angle_min: float = 0,
    angle_max: float = 2 * np.pi,
    angular_resolution: float = np.pi / 180,  # 1 degree
    max_range: float = 120.0,
) -> List[Tuple[int, int]]:
    """
    Ultra-optimized 2D LiDAR simulation using Bresenham's algorithm and NumPy vectorization.

    Args:
        grid_map (np.ndarray): Binary grid map where 0 = obstacle, 1 = free space.
        position (Tuple[float, float]): (x, y) position of the LiDAR sensor.
        angle_min (float): Starting angle of the scan in radians.
        angle_max (float): Ending angle of the scan in radians.
        angular_resolution (float): Angular resolution in radians.
        max_range (float): Maximum range of the sensor.

    Returns:
        List[Tuple[int, int]]: List of detected (x, y) points.
    """
    detected_points = []
    height, width = grid_map.shape
    x0, y0 = int(position[0]), int(position[1])

    # Validate LiDAR position
    if not (0 <= x0 < width and 0 <= y0 < height):
        return []

    # Precompute angles and step directions
    angles = np.arange(angle_min, angle_max, angular_resolution)
    cos_vals = np.cos(angles)
    sin_vals = np.sin(angles)

    # Bresenham's Algorithm for Ray Casting
    def ray_cast(dx: float, dy: float) -> Tuple[int, int]:
        """
        Cast a ray using an optimized Bresenham's algorithm.
        """
        x, y = x0, y0
        step_x = 1 if dx > 0 else -1
        step_y = 1 if dy > 0 else -1

        t_max_x = (1 - (x0 % 1)) / abs(dx) if dx != 0 else float("inf")
        t_max_y = (1 - (y0 % 1)) / abs(dy) if dy != 0 else float("inf")
        t_delta_x = abs(1 / dx) if dx != 0 else float("inf")
        t_delta_y = abs(1 / dy) if dy != 0 else float("inf")

        for _ in range(int(max_range)):  # Stop at max range
            if not (0 <= x < width and 0 <= y < height):
                break  # Out of bounds
            if grid_map[y, x] == 0:
                return x, y  # Hit an obstacle

            if t_max_x < t_max_y:
                t_max_x += t_delta_x
                x += step_x
            else:
                t_max_y += t_delta_y
                y += step_y

        return None  # No obstacle detected

    # Perform ray casting for all precomputed angles
    for dx, dy in zip(cos_vals, sin_vals):
        point = ray_cast(dx, dy)
        if point:
            detected_points.append(point)

    return detected_points


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
