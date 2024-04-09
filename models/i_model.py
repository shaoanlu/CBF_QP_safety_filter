from abc import abstractmethod

class ModelInterface:
    def __init__(self) -> None:
        pass

    @abstractmethod
    def foward(self):
        raise NotImplementedError
