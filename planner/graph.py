import math
import numpy as np

class Node:
     def __init__(self, pos: np.ndarray, parent=None, id=0):
        self.id = id  # Only for debugging
        self.pos = pos
        self.parent = parent
        if parent is None:
            self.connections = [self]
        else:
            self.connections = self.parent.connections + [self]

class Graph:
    def __init__(self, init_node = None) -> None:
        self.nodes = []
        if init_node != None:
            self.nodes.append(init_node)

    def add_node(self, point):
        self.nodes.append(point)


    @staticmethod
    def euclidean_metric(a, b):
        """Euclidean distance"""
        return math.sqrt(sum([(a[i] - b[i])**2 for i in range(len(a))]))

    def closest_node(self, point):
        min_dist = math.inf
        closest_node = None
        for node in self.nodes:
            dist = self.euclidean_metric(node.pos, point) 
            if  dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node