from abc import abstractmethod

class ModelInterface:
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
        raise NotImplementedError

    @abstractmethod
    def h_dot(self, *args, **kwargs):
        # derivative of the barrier function
        raise NotImplementedError
