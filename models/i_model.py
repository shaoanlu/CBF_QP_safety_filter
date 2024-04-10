from abc import abstractmethod

class ModelInterface:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def forward(self):
        raise NotImplementedError
