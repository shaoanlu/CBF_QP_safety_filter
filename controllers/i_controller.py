from abc import abstractmethod


class ControllerInterface:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def control(self, *args, **kwargs):
        raise NotImplementedError

    @abstractmethod
    def detect_collision(self, *args, **kwargs):
        raise NotImplementedError
