from abc import ABC, abstractmethod

class ControllerBase(ABC):
    @abstractmethod
    def compute_command(self, state, path):
        """
        :param state: State
        :param path: list of Point
        :return: acc, steer, target_point
        """
        pass
