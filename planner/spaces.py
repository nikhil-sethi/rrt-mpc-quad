"""
spaces.py

Workspaces and config spaces for the quadrotor

"""
import random
from scipy.stats import skewnorm
import numpy as np

class Space:
    def __init__(self, low, high) -> None:
        self.ll = low
        self.hl = high

        assert len(low) == len(high), "The lower limit and higher limit must be the same size!"

    def sample(self,goal:list,perc_to_end_goal):
        if random.random() < perc_to_end_goal:
            return goal
        else:
            return [self.ll[i] + random.random()*(self.hl[i]-self.ll[i]) for i in range(len(self.ll))]

    # def sample_biased(self, point, bias):
    #     skewnorm()

    #     if prob>bias


# class Workspace:
#     def __init__(self, bounds_x: np.ndarray, bounds_y: np.ndarray, bounds_z: np.ndarray, obstacles: list):
#         self.bounds_x = bounds_x
#         self.bounds_y = bounds_y
#         self.bounds_z = bounds_z
#         self.obstacles = obstacles
#         # Add the obstacles in this class  