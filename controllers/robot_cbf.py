import numpy as np
from scipy import sparse
import osqp
from typing import Optional

from models.i_model import ModelInterface
from controllers.i_controller import ControllerInterface


# IDs for detecting arrow keypress in PyGame
K_LEFT = 1073741904
K_RIGHT = 1073741903
K_UP = 1073741906
K_DOWN = 1073741905


class RobotCBF(ControllerInterface):
    def __init__(
        self,
        model: ModelInterface,
        color: tuple = (0, 0, 255),
        vel: float = 3,
        size: int = 30,
    ):
        """
        A controller for a robot using Control Barrier Functions (CBF) as
        a safety filter for collision prevention. The CBF modifies user commands
        ensuring safety without significantly deviating from the intended movement.
        """
        self.model = model
        self.ux = 0  # actual control input
        self.uy = 0  # actual control input
        self.nominal_ux = 0  # user control input
        self.nominal_uy = 0  # user control input
        self.color = color  # color of the robot in PyGame GUI
        self.vel = vel  # robot velocity generated by use control input
        self.size = size
        self.is_collided = False
        self.prob = None

    @property
    def x(self):
        return self.model.x[0]

    @property
    def y(self):
        return self.model.x[1]

    @x.setter
    def x(self, value):
        self.model.x[0] = value

    @y.setter
    def y(self, value):
        self.model.x[1] = value

    def control(
        self,
        key: Optional[int] = None,
        use_cbf: bool = False,
        force_direction_unchanged: bool = False,  # whether to allow safety filter to change direction of user input
        cbf_alpha: float = 1e-1,
        penalty_slack: float = 10,
        collision_objects: list = [],
        is_lidar_simulation: bool = False,
    ):
        """
        Processes control inputs, applies CBF for safety if enabled, and updates the robot's position.
        """
        self.nominal_ux, self.nominal_uy = 0, 0

        # get user command
        self._update_nominal_control(key)

        # get cbf filtered command
        if use_cbf:
            self._apply_cbf_safe_control(
                self.nominal_ux,
                self.nominal_uy,
                cbf_alpha,
                penalty_slack,
                force_direction_unchanged,
                collision_objects,
                is_lidar_simulation,
            )
        else:
            self._apply_nominal_control()

        # update xy positions based on derived command
        self._update_positions(self.ux, self.uy)

    def _apply_nominal_control(self):
        """
        Applies nominal control inputs (user inputs) directly to the robot without safety adjustments.
        """
        self.ux, self.uy = self.nominal_ux, self.nominal_uy

    def _update_positions(self, ux: float, uy: float):
        model_control = np.array([ux, uy])
        self.model.forward(model_control)

    def _update_nominal_control(self, key: int):
        """
        Updates nominal control inputs based on user key input.
        """
        ux, uy = 0, 0
        if key is not None:
            if key == K_LEFT:
                ux = -self.vel
            elif key == K_RIGHT:
                ux = self.vel
            elif key == K_UP:
                uy = -self.vel
            elif key == K_DOWN:
                uy = self.vel
        self.nominal_ux, self.nominal_uy = ux, uy

    def _apply_cbf_safe_control(
        self,
        ux: float,
        uy: float,
        cbf_alpha: float,
        penalty_slack: float,
        force_direction_unchanged: bool,
        collision_objects: list,
        is_lidar_simulation: bool,
    ):
        """
        Calculate the safe command by solveing the following optimization problem

                    minimize  || u - u_nom ||^2 + k * δ
                      u, δ
                    s.t.
                            h'(x) ≥ -𝛼 * h(x) - δ
                            u_min ≤ u ≤ u_max
                                0 ≤ δ ≤ inf
        where
            u = [ux, uy] is the control input in x and y axis respectively.
            δ is the slack variable
            h(x) is the control barrier function and h'(x) its derivative

        The problem above can be formulated as QP (ref: https://osqp.org/docs/solver/index.html)

                    minimize 1/2 * x^T * Px + q^T x
                        x
                    s.t.
                                l ≤ Ax ≤ u
        where
            x = [ux, uy, δ]

        """
        # Calculate barrier values and coeffs in h_dot
        if not is_lidar_simulation:
            h, coeffs_dhdx = self._calculate_h_and_coeffs_dhdx(collision_objects)
        else:
            h, coeffs_dhdx = self._calculate_composite_h_and_coeffs_dhdx(collision_objects, cbf_alpha)

        # Define problem data
        P, q, A, l, u = self._define_QP_problem_data(
            ux, uy, cbf_alpha, penalty_slack, h, coeffs_dhdx, force_direction_unchanged
        )

        self.prob = osqp.OSQP()
        self.prob.setup(P, q, A, l, u, verbose=False, time_limit=0)

        # Solve QP problem
        res = self.prob.solve()
        ux, uy, _ = res.x

        # Handle infeasible sol.
        ux = self.nominal_ux if ux is None else ux
        uy = self.nominal_uy if uy is None else uy

        self.ux, self.uy = ux, uy

    def _calculate_h_and_coeffs_dhdx(self, collision_objects: list):
        h = []  # barrier values (here, remaining distance to each obstacle)
        coeffs_dhdx = []  # dhdt = dhdx * dxdt = dhdx * u
        for obj in collision_objects:
            model_state = [self.x, self.y]
            self.model.update_params({"xr": obj.x, "yr": obj.y, "size": self.size})
            h.append(self.model.h(model_state))
            # Note: append additional elements for the slack variable δ
            coeffs_dhdx.append(self.model.h_dot(model_state) + [1])

            # NOTE: To speedup computation, we can calculate dhdu offline and hardcoded it as below
            # h.append((self.x - obj.x) ** 2 + (self.y - obj.y) ** 2 - (self.size * 2.3) ** 2)
            # coeffs_dhdx.append([2 * self.x - 2 * obj.x, 2 * self.y - 2 * obj.y, penalty_slack])
        return h, coeffs_dhdx

    def _calculate_composite_h_and_coeffs_dhdx(self, collision_objects: list, cbf_alpha: float):
        def log_sum_exp(x):
            return np.log(np.sum(np.exp(x), axis=0))

        if len(collision_objects) == 0:
            return [1], [[0, 0, 1]]

        h = []
        coeffs_dhdx = []
        kappa, dist_buffer = 5e-2 * cbf_alpha, self.size * 1.3
        x0 = np.array([self.x, self.y])
        lidar_points = np.array(collision_objects)
        hi_x = np.linalg.norm(x0 - lidar_points, axis=1) ** 2 - dist_buffer**2
        assert hi_x.shape == (len(lidar_points),), hi_x
        h_x = -1 / kappa * log_sum_exp(-kappa * hi_x)  # beware of numerical error due to exponent
        dhdx = np.sum(np.exp(-kappa * (hi_x - h_x))[..., None] * (-2 * lidar_points + 2 * x0), axis=0)
        # h_x = -1 / kappa * log_sump_exp(-kappa * np.tanh(hi_x))  # scale to prevent numerical error
        # dhdx = np.sum(
        #     np.exp(-kappa * (hi_x - h_x))[..., None] * 2 * (x0 - lidar_points) / (np.cosh(hi_x)[..., None] ** 2),
        #     axis=0,
        # )  # this drivative is buggy
        assert dhdx.shape == (2,), dhdx
        h.append(h_x)
        coeffs_dhdx.append([dhdx[0], dhdx[1], 1])
        return h, coeffs_dhdx

    def _define_QP_problem_data(
        self,
        ux: float,
        uy: float,
        cbf_alpha: float,
        penalty_slack: float,
        h: np.ndarray,
        coeffs_dhdx: np.ndarray,
        force_direction_unchanged: bool,
    ):
        # P: shape (nx, nx)
        # q: shape (nx,)
        # A: shape (nh+nx, nx)
        # l: shape (nh+nx,)
        # u: shape (nh+nx,)
        # (nx: number of state; nh: number of control barrier functions)
        P = sparse.csc_matrix([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
        q = np.array([-ux, -uy, penalty_slack])
        A = sparse.csc_matrix([c for c in coeffs_dhdx] + [[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        if force_direction_unchanged:
            l = np.array([-cbf_alpha * h_ for h_ in h] + [np.minimum(ux, 0), np.minimum(uy, 0), 0])
            u = np.array([np.inf for _ in h] + [np.maximum(ux, 0), np.maximum(uy, 0), np.inf])
        else:
            l = np.array([-cbf_alpha * h_ for h_ in h] + [-self.vel, -self.vel, 0])
            u = np.array([np.inf for _ in h] + [self.vel, self.vel, np.inf])
        return P, q, A, l, u

    def detect_collision(self, collision_objects: list = []):
        """
        Loop through all objects (robots) and set self.is_collides as True
        if distance to any specified robot being smaller than 2*self.size
        """
        self.is_collided = False
        if len(collision_objects) > 0:
            for obj in collision_objects:
                dist = np.linalg.norm(np.array([self.x, self.y] - np.array([obj.x, obj.y])))
                if dist <= (self.size * 2):
                    self.is_collided = True
