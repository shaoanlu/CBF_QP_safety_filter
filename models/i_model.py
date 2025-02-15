from abc import abstractmethod
from typing import Any


class ModelInterface:
    """
    Interface for a model of a control affine system.

    A control affine system is defined by the following equations:
        x_dot = f(x) + g(x) * u

    where x is the state (of dim n), u is the control input (of dim m), and x_dot is the state derivative.
    """

    def __init__(self, *args, **kwargs) -> None:
        pass

    @abstractmethod
    def forward(self, *args, **kwargs):
        raise NotImplementedError

    @abstractmethod
    def update_params(self, *args, **kwargs):
        raise NotImplementedError

    @abstractmethod
    def h(self, *args, **kwargs):
        # barrier function
        # R^n -> R^1
        raise NotImplementedError

    @abstractmethod
    def h_dot(self, *args, **kwargs):
        # derivative of the barrier function
        raise NotImplementedError

    @abstractmethod
    def f_x(self, x: Any) -> Any:
        # f(x) of a control affine system: x_dot = f(x) + g(x) * u
        # R^n -> R^n
        raise NotImplementedError

    @abstractmethod
    def g_x(self, x: Any) -> Any:
        # g(x) of a control affine system: x_dot = f(x) + g(x) * u
        # R^n -> R^n*m
        raise NotImplementedError
