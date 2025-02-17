# tests/test_controllers.py
import unittest
import numpy as np
from scipy import sparse
from controllers.robot_cbf import RobotCBF, K_LEFT, K_RIGHT, K_UP, K_DOWN
from models.i_model import ModelInterface
from models.i_model import ModelInterface
from unittest.mock import MagicMock


class DummySimpleRobotDynamics(ModelInterface):
    def __init__(self, x0):
        self.x = x0.astype(np.float64)

    def forward(self, u):
        self.x += u

    def f_x(self, x):  # Added
        return np.zeros_like(x)

    def g_x(self, x):  # Added
        return np.eye(len(x))

    def h(self, x):
        return 0.0

    def h_dot(self, x):
        return [0.0, 0.0]

    def update_params(self, p):
        pass


class TestRobotCBF(unittest.TestCase):
    def setUp(self):
        self.simple_robot_dynamics = DummySimpleRobotDynamics(np.array([0, 0]))
        self.robot = RobotCBF(self.simple_robot_dynamics)
        self.mock_model = MagicMock(spec=ModelInterface)
        self.mock_model.x = np.array([0.0, 0.0])
        self.mock_model.f_x.return_value = np.zeros((2,))
        self.mock_model.g_x.return_value = np.eye(2)
        self.robot_with_mock = RobotCBF(self.mock_model)

    def test_init(self):
        # XY position equals given initial value (0, 0)
        self.assertEqual(self.robot.x, 0)
        self.assertEqual(self.robot.y, 0)
        self.assertEqual(self.robot_with_mock.model, self.mock_model)

    def test_x_y_properties(self):
        self.robot.x = 5
        self.robot.y = 10
        self.assertEqual(self.robot.x, 5)
        self.assertEqual(self.robot.y, 10)
        self.assertTrue(np.all(self.robot.model.x == np.array([5, 10])))

    def test_update_positions(self):
        # Given new xY position
        new_x, new_y = 1, 1
        # When update
        self.robot._update_positions(new_x, new_y)
        # Then new robot position equals exptected values
        self.assertEqual(self.robot.x, 1)
        self.assertEqual(self.robot.y, 1)

    def test_update_nominal_control(self):
        # When update control command
        self.robot._update_nominal_control(K_LEFT)
        # Then robot nominal control input equals the expected value
        self.assertEqual(self.robot.nominal_ux, -self.robot.vel)
        self.assertEqual(self.robot.nominal_uy, 0)

        self.robot._update_nominal_control(K_RIGHT)
        self.assertEqual(self.robot.nominal_ux, self.robot.vel)
        self.assertEqual(self.robot.nominal_uy, 0)

        self.robot._update_nominal_control(K_UP)
        self.assertEqual(self.robot.nominal_ux, 0)
        self.assertEqual(self.robot.nominal_uy, -self.robot.vel)

        self.robot._update_nominal_control(K_DOWN)
        self.assertEqual(self.robot.nominal_ux, 0)
        self.assertEqual(self.robot.nominal_uy, self.robot.vel)

        self.robot._update_nominal_control(None)
        self.assertEqual(self.robot.nominal_ux, 0)
        self.assertEqual(self.robot.nominal_uy, 0)

    def test_apply_nominal_control(self):
        # When update and apply control command
        self.robot._update_nominal_control(K_LEFT)
        self.robot._apply_nominal_control()
        # Then robot control input equals the expected value
        self.assertEqual(self.robot.ux, -self.robot.vel)
        self.assertEqual(self.robot.uy, 0)

    def test_detect_collision(self):
        # Given two robots in the env
        robot1 = RobotCBF(DummySimpleRobotDynamics(np.array([30, 0])))
        robot2 = RobotCBF(DummySimpleRobotDynamics(np.array([0, 70])))
        collision_objects = [robot1, robot2]
        # When check collision (distance to robot1 <= robot_size = 30)
        self.robot.detect_collision(collision_objects)
        # Then is_collided
        self.assertTrue(self.robot.is_collided)

        # When check collision (distance to all robots > robot_size)
        self.robot._update_positions(100, 0)
        self.robot.detect_collision(collision_objects)
        # Then no collision
        self.assertFalse(self.robot.is_collided)

        self.robot.detect_collision([])
        self.assertFalse(self.robot.is_collided)

    def test_control_no_key(self):
        self.robot.control()
        self.assertEqual(self.robot.ux, 0)
        self.assertEqual(self.robot.uy, 0)

    def test_control_with_key(self):
        self.robot.control(K_LEFT)
        self.assertEqual(self.robot.ux, -self.robot.vel)
        self.assertEqual(self.robot.uy, 0)

    def test_apply_cbf_safe_control_no_objects(self):
        self.robot._apply_cbf_safe_control(
            ux=1, uy=1, cbf_alpha=0.1, penalty_slack=10, force_direction_unchanged=False, collision_objects=[]
        )
        self.assertNotEqual(self.robot.ux, 0.0)
        self.assertNotEqual(self.robot.uy, 0.0)

    def test_get_control_bounds(self):
        control = np.array([2, -3])
        max_control, min_control = self.robot._get_control_bounds(control, False)
        self.assertTrue(np.array_equal(max_control, np.array([self.robot.vel, self.robot.vel])))
        self.assertTrue(np.array_equal(min_control, np.array([-self.robot.vel, -self.robot.vel])))

        max_control, min_control = self.robot._get_control_bounds(control, True)  # Force direction
        self.assertTrue(np.array_equal(max_control, np.array([2, 0])))
        self.assertTrue(np.array_equal(min_control, np.array([0, -3])))

    def test_solve_cbf_qp_osqp(self):
        P = np.eye(3)
        q = np.array([1, 2, 3])
        A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        l = np.array([-1, -1, -1])
        u = np.array([1, 1, 1])
        result = self.robot._solve_qp_osqp(sparse.csc_matrix(P), q, sparse.csc_matrix(A), l, u)
        self.assertIsNotNone(result)

    def test_calculate_h_and_coeffs_dhdx(self):
        # Given two robots in the env
        robot1 = RobotCBF(DummySimpleRobotDynamics(np.array([30, 0])))
        robot2 = RobotCBF(DummySimpleRobotDynamics(np.array([0, 70])))
        collision_objects = [robot1, robot2]

        # When calculate CBF params
        h, coeffs_dhdx = self.robot_with_mock._calculate_h_and_coeffs_dhdx(collision_objects)

        # Check the results
        self.assertEqual(len(h), len(collision_objects))
        self.assertEqual(len(coeffs_dhdx), len(collision_objects))
        self.mock_model.h_dot.assert_called()
        self.mock_model.update_params.assert_called()
        self.mock_model.h.assert_called()

    def test_calculate_composite_h_and_coeffs_dhdx(self):
        # When no collision objects
        h, coeffs_dhdx = self.robot_with_mock._calculate_composite_h_and_coeffs_dhdx([])
        self.assertEqual(h, [1])
        self.assertEqual(coeffs_dhdx, [[0, 0, 1]])

        # When some collision objects
        collision_objects = [[1, 1], [2, 2]]
        h, coeffs_dhdx = self.robot_with_mock._calculate_composite_h_and_coeffs_dhdx(collision_objects)
        self.assertEqual(len(h), 1)
        self.assertEqual(len(coeffs_dhdx), 1)
        self.assertEqual(len(coeffs_dhdx[0]), 3)

    def test_estimate_disturbance(self):
        # Mock necessary methods and attributes
        h = [0.5]  # Example barrier value
        coeffs_dhdx = [[1.0, 0.5, 0.0]]  # Example coefficients
        self.robot_with_mock.ux, self.robot_with_mock.uy = 0.1, 0.2

        # Call _estimate_disturbance
        disturbance = self.robot_with_mock._estimate_disturbance(h=h, coeffs_dhdx=coeffs_dhdx)
        self.assertIsInstance(disturbance, float)

    def test_apply_cbf_safe_control_lidar_simulation(self):
        self.robot_with_mock._apply_cbf_safe_control(
            ux=1.0,
            uy=1.0,
            cbf_alpha=0.1,
            penalty_slack=10,
            force_direction_unchanged=False,
            collision_objects=[[1.0, 1.0]],  # Example lidar point
            is_lidar_simulation=True,
        )
        self.mock_model.f_x.assert_called()
        self.mock_model.g_x.assert_called()

    def test_solve_qp_osqp_failure(self):
        P = np.eye(3)
        q = np.array([1, 2, 3])
        A = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        l = np.array([1, 1, 1])  # Make it infeasible
        u = np.array([-1, -1, -1])

        # NOTE: need better error handling. currently, osqp might raise runtime error instead of returning None
        with self.assertRaises(ValueError):
            _ = self.robot._solve_qp_osqp(sparse.csc_matrix(P), q, sparse.csc_matrix(A), l, u)

    def test_control_with_cbf(self):
        # Test with a simple obstacle to ensure CBF is activated
        self.robot.control(
            K_RIGHT, use_cbf=True, collision_objects=[RobotCBF(DummySimpleRobotDynamics(np.array([30, 0])))]
        )  # in collision range of ego robot
        self.assertNotEqual(self.robot.ux, self.robot.vel)  # Expecting a modified control due to CBF


if __name__ == "__main__":
    unittest.main()
