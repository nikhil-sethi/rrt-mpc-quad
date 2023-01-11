import numpy as np
class Pose():
    def __init__(self, origin, orient) -> None:
        self.origin = np.array(origin) # xyz
        self.orient = np.array(orient) # rpy