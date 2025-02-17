# tests/test_models.py
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

        model2 = SimpleRobotDynamics(x0=np.array([1, 1]), xr=2, yr=3, size=40)
        self.assertTrue(np.all(model2.x == np.array([1, 1])))
        self.assertEqual(model2.xr, 2)
        self.assertEqual(model2.yr, 3)
        self.assertEqual(model2.size, 40)

    def test_forward(self):
        self.model.forward(np.array([1, 2]))
        self.assertTrue(np.all(self.model.x == np.array([1, 2])))

    def test_x_dot(self):
        u = np.array([1.5, 2.5])
        x_dot_result = self.model.x_dot(u)
        self.assertTrue(np.allclose(x_dot_result, u))  # Use np.allclose for float comparison

        u = np.array([-1, -5])
        x_dot_result = self.model.x_dot(u)
        self.assertTrue(np.allclose(x_dot_result, u))

    def test_f_x(self):
        self.assertTrue(np.array_equal(self.model.f_x(np.array([1, 1])), np.array([0, 0])))

    def test_g_x(self):
        self.assertTrue(np.array_equal(self.model.g_x(np.array([1, 1])), np.array([[1, 0], [0, 1]])))

    def test_h(self):
        # test the hard-coded barrier function h
        h_val = self.model.h(np.array([0, 0]))
        self.assertEqual(h_val, -((self.model.size * 2.3) ** 2))

        self.model.xr = 1
        self.model.yr = 1
        h_val = self.model.h(np.array([1, 1]))
        self.assertEqual(h_val, -((self.model.size * 2.3) ** 2))

        h_val = self.model.h([1, 1])
        self.assertEqual(h_val, -((self.model.size * 2.3) ** 2))

    def test_h_dot(self):
        h_dot_val = self.model.h_dot(np.array([0, 0]))
        self.assertTrue(np.all(h_dot_val == np.array([0, 0])))

        self.model.xr = 1
        self.model.yr = 1
        h_dot_val = self.model.h_dot(np.array([1, 1]))
        self.assertTrue(np.all(h_dot_val == np.array([0, 0])))

        h_dot_val = self.model.h_dot([1, 1])
        self.assertTrue(np.all(h_dot_val == np.array([0, 0])))

        self.model.xr = 2
        self.model.yr = 3
        h_dot_val = self.model.h_dot(np.array([1, 1]))
        self.assertTrue(np.all(h_dot_val == np.array([-2, -4])))

    def test_update_params(self):
        params = {"xr": 10, "yr": 20, "size": 40}
        self.model.update_params(params)
        self.assertEqual(self.model.xr, 10)
        self.assertEqual(self.model.yr, 20)
        self.assertEqual(self.model.size, 40)


if __name__ == "__main__":
    unittest.main()
