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
        self.bounds_x = np.array([low[0],high[0]])
        self.bounds_y = np.array([low[1],high[1]])
        self.bounds_z = np.array([low[2],high[2]])