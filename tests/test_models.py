import unittest
import numpy as np
from models.robot_dynamics import SimpleRobotDynamics


class TestSimpleRobotModel(unittest.TestCase):
    def setUp(self):
        self.model = SimpleRobotDynamics(np.array([0, 0]))

    def test_init(self):
        self.assertTrue(np.all(self.model.x == np.array([0, 0])))
        self.assertEqual(self.model.xr, 0)
        self.assertEqual(self.model.yr, 0)
        self.assertEqual(self.model.size, 30)

    def test_forward(self):
        self.model.forward(np.array([1, 2]))
        self.assertTrue(np.all(self.model.x == np.array([1, 2])))

    def test_x_dot(self):
        x_dot = self.model.x_dot(np.array([1, 2]))
        self.assertTrue(np.all(x_dot == np.array([1, 2])))

    def test_h(self):
        h_val = self.model.h(np.array([0, 0]))
        self.assertEqual(h_val, -((self.model.size * 2.3) ** 2))

    def test_h_dot(self):
        h_dot_val = self.model.h_dot(np.array([0, 0]))
        self.assertTrue(np.all(h_dot_val == np.array([0, 0])))

    def test_update_params(self):
        params = {"xr": 10, "yr": 20, "size": 40}
        self.model.update_params(params)
        self.assertEqual(self.model.xr, 10)
        self.assertEqual(self.model.yr, 20)
        self.assertEqual(self.model.size, 40)


if __name__ == "__main__":
    unittest.main()
