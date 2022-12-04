"""
spaces.py

Workspaces and config spaces for the quadrotor

"""
import random

class Space:
    def __init__(self, low, high) -> None:
        self.ll = low
        self.hl = high

        assert low == high, "The lower limit and higher limit must be the same size!"

    def sample(self):
        return [self.ll[i] + random.random()*(self.hl[i]-self.ll[i]) for i in range(self.ll)]

    