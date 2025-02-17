import unittest
import numpy as np
from sensors.lidar import simulate_2d_lidar_scan, create_obstacle_binary_map


class TestLidarSimulation(unittest.TestCase):
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a simple test map with known obstacles
        self.test_width = 100
        self.test_height = 100
        self.test_centers = [(50, 50)]
        self.test_radii = [10]
        self.test_map = create_obstacle_binary_map(
            self.test_width, self.test_height, self.test_centers, self.test_radii
        )

    def test_lidar_scan_basic_functionality(self):
        """Test basic functionality of LiDAR scan with default parameters."""
        position = (25, 25)
        points = simulate_2d_lidar_scan(self.test_map, position)

        self.assertIsInstance(points, list)
        self.assertTrue(all(isinstance(p, tuple) and len(p) == 2 for p in points))
        self.assertTrue(all(isinstance(c, int) for p in points for c in p))

    def test_lidar_scan_out_of_bounds(self):
        """Test LiDAR scan with out-of-bounds position."""
        invalid_positions = [(-1, -1), (self.test_width + 1, self.test_height + 1), (-1, 50), (50, -1)]

        for position in invalid_positions:
            points = simulate_2d_lidar_scan(self.test_map, position)
            self.assertEqual(points, [], f"Expected empty list for position {position}")

    def test_lidar_scan_angle_range(self):
        """Test LiDAR scan with different angle ranges."""
        position = (25, 25)

        # Test 90-degree scan
        points_90deg = simulate_2d_lidar_scan(self.test_map, position, angle_min=0, angle_max=np.pi / 2)

        # Test 180-degree scan
        points_180deg = simulate_2d_lidar_scan(self.test_map, position, angle_min=0, angle_max=np.pi)

        self.assertGreater(len(points_180deg), len(points_90deg))

    def test_lidar_scan_resolution(self):
        """Test LiDAR scan with different angular resolutions."""
        position = (25, 25)

        # Test with coarse resolution
        points_coarse = simulate_2d_lidar_scan(self.test_map, position, angular_resolution=np.pi / 18)  # 10 degrees

        # Test with fine resolution
        points_fine = simulate_2d_lidar_scan(self.test_map, position, angular_resolution=np.pi / 180)  # 1 degree

        self.assertGreater(len(points_fine), len(points_coarse))

    def test_lidar_scan_max_range(self):
        """Test LiDAR scan with different maximum ranges."""
        position = (25, 25)

        # Test with short range
        points_short = simulate_2d_lidar_scan(self.test_map, position, max_range=10.0)

        # Test with long range
        points_long = simulate_2d_lidar_scan(self.test_map, position, max_range=120.0)

        self.assertGreaterEqual(len(points_long), len(points_short))


class TestObstacleMapCreation(unittest.TestCase):
    def test_create_basic_map(self):
        """Test creation of basic obstacle map."""
        width, height = 50, 50
        centers = [(25, 25)]
        radii = [5]

        grid = create_obstacle_binary_map(width, height, centers, radii)

        self.assertEqual(grid.shape, (height, width))
        self.assertEqual(grid.dtype, np.int_)
        self.assertTrue(np.all(grid[0:3, :] == 0))  # Check border
        self.assertTrue(np.all(grid[-3:, :] == 0))  # Check border
        self.assertTrue(np.all(grid[:, 0:3] == 0))  # Check border
        self.assertTrue(np.all(grid[:, -3:] == 0))  # Check border

    def test_multiple_obstacles(self):
        """Test creation of map with multiple obstacles."""
        width, height = 100, 100
        centers = [(25, 25), (75, 75), (50, 50)]
        radii = [10, 15, 5]

        grid = create_obstacle_binary_map(width, height, centers, radii)

        # Check that obstacles exist (some points should be 0)
        self.assertTrue(np.any(grid == 0))
        # Check that free space exists (some points should be 1)
        self.assertTrue(np.any(grid == 1))

    def test_overlapping_obstacles(self):
        """Test creation of map with overlapping obstacles."""
        width, height = 50, 50
        centers = [(25, 25), (26, 26)]  # Overlapping centers
        radii = [5, 5]

        grid = create_obstacle_binary_map(width, height, centers, radii)

        # The overlapping region should still be marked as obstacle (0)
        center_region = grid[20:30, 20:30]
        self.assertTrue(np.any(center_region == 0))

    def test_edge_cases(self):
        """Test edge cases for obstacle map creation."""
        # Test empty obstacles
        grid = create_obstacle_binary_map(10, 10, [], [])
        self.assertTrue(np.all(grid[3:-3, 3:-3] == 1))

        # Test single-cell map
        grid = create_obstacle_binary_map(1, 1, [], [])
        self.assertEqual(grid.shape, (1, 1))

        # Test obstacles larger than map
        grid = create_obstacle_binary_map(20, 20, [(10, 10)], [50])
        self.assertTrue(np.any(grid == 0))


if __name__ == "__main__":
    unittest.main()
