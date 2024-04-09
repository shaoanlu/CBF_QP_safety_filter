from abc import abstractmethod


class ControllerInterface:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def control(self):
        raise NotImplementedError

    @abstractmethod
    def detect_collision(self):
        raise NotImplementedError