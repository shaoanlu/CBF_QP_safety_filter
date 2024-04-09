import unittest
import numpy as np
from controllers.robot_cbf import RobotCBF, K_LEFT, K_RIGHT, K_UP, K_DOWN


class DummySimpleRobotDynamics:
    def __init__(self, x0) -> None:
        self.x = x0

    def forward(self, u):
        self.x += u


class TestRobot(unittest.TestCase):
    def setUp(self):
        self.simple_robot_dynamics = DummySimpleRobotDynamics(np.array([0, 0]))
        self.robot = RobotCBF(self.simple_robot_dynamics)

    def test_init(self):
        self.assertEqual(self.robot.x, 0)
        self.assertEqual(self.robot.y, 0)

    def test_update_positions(self):
        ux, uy = 1, 1
        self.robot._update_positions(ux, uy)
        self.assertEqual(self.robot.x, 1)
        self.assertEqual(self.robot.y, 1)

    def test_update_nominal_control(self):
        self.robot._update_nominal_control(K_LEFT)
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

    def test_apply_nominal_control(self):
        self.robot._update_nominal_control(K_LEFT)
        self.robot._apply_nominal_control()
        self.assertEqual(self.robot.ux, -self.robot.vel)
        self.assertEqual(self.robot.uy, 0)

    def test_detect_collision(self):
        obj1 = RobotCBF(DummySimpleRobotDynamics(np.array([30, 0])))
        obj2 = RobotCBF(DummySimpleRobotDynamics(np.array([0, 70])))
        collision_objects = [obj1, obj2]

        self.robot.detect_collision(collision_objects)
        self.assertTrue(self.robot.is_collided)

        self.robot._update_positions(100, 0)
        self.robot.detect_collision(collision_objects)
        self.assertFalse(self.robot.is_collided)


if __name__ == "__main__":
    unittest.main()
