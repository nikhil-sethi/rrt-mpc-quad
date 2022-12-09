import math
import numpy as np

class Node:
     def __init__(self, pos: np.ndarray, parent=None, id=0):
        self.id = id  # Only for debugging
        self.pos = pos
        self.parent = parent
        if parent is None:
            self.connections = [self]
            self.dist_from_start = 0
        else:
            self.connections = self.parent.connections + [self]
            self.dist_from_start = self.parent.dist_from_start + np.linalg.norm(self.pos - self.parent.pos)

class Graph:
    def __init__(self, start_node: Node):
        self.nodes = [start_node]
    
    def add_node(self, node: Node):
        self.nodes.append(node)
    
    def remove_node(self, node: Node):
        nodes_arr = np.array(self.nodes)
        nodes_arr = np.delete(nodes_arr, np.sum((np.array([node_s.id for node_s in self.nodes])==node.id)*np.arange(len(self.nodes))))
        self.nodes = list(nodes_arr)