import unittest
import numpy as np
from controllers.robot_cbf import RobotCBF, K_LEFT, K_RIGHT, K_UP, K_DOWN


class DummySimpleRobotDynamics:
    def __init__(self, x0) -> None:
        self.x = x0

    def forward(self, u):
        self.x += u


class TestRobotCBF(unittest.TestCase):
    def setUp(self):
        self.simple_robot_dynamics = DummySimpleRobotDynamics(np.array([0, 0]))
        self.robot = RobotCBF(self.simple_robot_dynamics)

    def test_init(self):
        # XY position equals given initial value (0, 0)
        self.assertEqual(self.robot.x, 0)
        self.assertEqual(self.robot.y, 0)

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


if __name__ == "__main__":
    unittest.main()
