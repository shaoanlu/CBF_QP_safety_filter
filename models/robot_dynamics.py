# import torch
import numpy as np
from typing import Union, List, Dict

from models.i_model import ModelInterface


class SimpleRobotDynamics(ModelInterface):
    def __init__(self, x0: np.ndarray, xr: float = 0, yr: float = 0, size: int = 30) -> None:
        self.x = x0.astype(np.float64)  # xy positoin of the robot, shape(2,)
        self.xr = xr  # x position of the obstacle
        self.yr = yr  # y position of the obstacle
        self.size = size

    def forward(self, u: np.ndarray) -> None:
        self.x += self.x_dot(u)

    def x_dot(self, u: np.ndarray) -> np.ndarray:
        return u.astype(np.float64)

    def f_x(self, x: np.ndarray) -> np.ndarray:
        return np.zeros((2,))

    def g_x(self, x: np.ndarray) -> np.ndarray:
        return np.eye(2)

    def h(self, x: Union[List, np.ndarray]) -> float:
        return (x[0] - self.xr) ** 2 + (x[1] - self.yr) ** 2 - (self.size * 2.3) ** 2

    def h_dot(self, x: Union[List, np.ndarray]) -> List:
        # inputs = tuple([torch.Tensor([ele]) for ele in x])
        # jacob = torch.autograd.functional.jacobian(
        #     func=lambda x, y: (x - self.xr) ** 2 + (y - self.yr) ** 2 - (self.size * 2.3) ** 2,
        #     inputs=inputs,
        # )
        # return [float(x.detach().numpy().copy()) for x in jacob]
        jacob_x = 2 * (x[0] - self.xr)
        jacob_y = 2 * (x[1] - self.yr)
        return [jacob_x, jacob_y]

    def update_params(self, p: Dict) -> None:
        self.xr = p["xr"]
        self.yr = p["yr"]
        self.size = p["size"]
