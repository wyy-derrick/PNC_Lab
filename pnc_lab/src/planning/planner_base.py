from abc import ABC, abstractmethod

class PlannerBase(ABC):
    @abstractmethod
    def plan(self, start, goal, obstacles):
        """
        :param start: Point
        :param goal: Point
        :param obstacles: list of (x, y, r)
        :return: list of Point
        """
        pass
