from abc import ABC
from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np

from controllers.caution_adam import CautionAdam


class DisturbanceEstimationStrategy(ABC):
    """Protocol defining the interface for disturbance estimation strategies."""

    def estimate(self, h: List[float], coeffs_dhdx: List[List[float]], ux: float, uy: float, **kwargs) -> float:
        """Estimate the disturbance based on the current state and inputs.

        Args:
            h: Current state measurement
            coeffs_dhdx: Coefficients for partial derivatives
            ux: Control input in x direction
            uy: Control input in y direction
            **kwargs: Additional keyword arguments for specific implementations

        Returns:
            float: the estimated disturbance
        """
        raise NotImplementedError


class BasicDisturbanceObserver(DisturbanceEstimationStrategy):
    """
    Implementation of a basic disturbance observer using auxiliary states.

    Reference:
        Alan, Anil, et al. "Disturbance observers for robust safety-critical control with control barrier functions."
        IEEE Control Systems Letters 7 (2022): 1123-1128.
    """

    def __init__(self, gain: float = 1.0):
        self.gain = gain
        self.aux_state = 0.0
        self.est_state = 0.0

    def estimate(
        self,
        h: List[float],
        coeffs_dhdx: List[List[float]],
        control: np.ndarray,
        f_x: np.ndarray,
        g_x: np.ndarray,
        **kwargs
    ) -> float:
        velocity = kwargs.get("velocity", 1.0)

        Lfh = np.array([coeffs_dhdx[0][:-1]]) @ f_x  # ignore last element of coeffs_dhdx which is for slack variable
        Lgh = np.array([coeffs_dhdx[0][:-1]]) @ g_x  # ignore last element of coeffs_dhdx which is for slack variable

        # Update auxiliary state
        self.aux_state += float(self.gain * (Lfh + Lgh @ control + self.est_state))

        # Update estimation state
        self.est_state = float(self.gain * h[0] - self.aux_state)

        # Clip the disturbance estimate
        max_disturbance = abs(coeffs_dhdx[0][0]) * velocity + abs(coeffs_dhdx[0][1]) * velocity
        self.est_state = np.clip(self.est_state, -max_disturbance, max_disturbance)

        return self.est_state


class CautionAdamDisturbanceObserver(DisturbanceEstimationStrategy):
    """
    Implementation of disturbance observer using CautionAdam optimization.

    Reference:
        Liang, Kaizhao, et al. "Cautious optimizers: Improving training with one line of code."
        arXiv preprint arXiv:2411.16085 (2024).
    """

    def __init__(self, lr: float = 1.0):
        self.optimizer = CautionAdam(lr=lr)
        self.prev_h: Optional[float] = None
        self.disturbance_xy = {"ux": 0.0, "uy": 0.0}

    def estimate(self, h: List[float], coeffs_dhdx: List[List[float]], control: np.ndarray, **kwargs) -> float:
        velocity = kwargs.get("velocity", 1.0)

        if self.prev_h is None:
            self.prev_h = h[0]

        # Compute predicted state derivative
        est_h_dot = coeffs_dhdx[0][0] * (control[0] + self.disturbance_xy["ux"]) + coeffs_dhdx[0][1] * (
            control[1] + self.disturbance_xy["uy"]
        )
        est_h = self.prev_h + est_h_dot

        # Update parameters using CautionAdam
        grads = {
            "ux": -2 * np.mean((h[0] - est_h) * coeffs_dhdx[0][0]),
            "uy": -2 * np.mean((h[0] - est_h) * coeffs_dhdx[0][1]),
        }

        self.disturbance_xy = self.optimizer.update(self.disturbance_xy, grads)

        # Clip the disturbance estimates
        self.disturbance_xy = {
            "ux": np.clip(self.disturbance_xy["ux"], -velocity, velocity),
            "uy": np.clip(self.disturbance_xy["uy"], -velocity, velocity),
        }

        # Compute final disturbance estimate
        disturbance = coeffs_dhdx[0][0] * self.disturbance_xy["ux"] + coeffs_dhdx[0][1] * self.disturbance_xy["uy"]

        self.prev_h = h[0]

        return disturbance


class DisturbanceObserver:
    """Main disturbance observer class that manages different estimation strategies."""

    def __init__(self, strategy: Optional[DisturbanceEstimationStrategy] = None):
        """Initialize the disturbance observer.

        Args:
            strategy: Initial estimation strategy to use
        """
        self.strategy = strategy or BasicDisturbanceObserver()

    def update(
        self,
        h: List[float],
        coeffs_dhdx: List[List[float]],
        control: np.ndarray,
        f_x: np.ndarray,
        g_x: np.ndarray,
        **kwargs
    ) -> float:
        """Update the disturbance estimation.

        Args:
            h: Current state measurement
            coeffs_dhdx: Coefficients for partial derivatives
            ux: Control input in x direction
            uy: Control input in y direction
            **kwargs: Additional parameters passed to the strategy

        Returns:
            float: the estimated disturbance
        """
        return self.strategy.estimate(h=h, coeffs_dhdx=coeffs_dhdx, control=control, f_x=f_x, g_x=g_x, **kwargs)

    def set_strategy(self, strategy: DisturbanceEstimationStrategy) -> None:
        """Set the disturbance estimation strategy.

        Args:
            strategy: New estimation strategy to use
        """
        self.strategy = strategy
