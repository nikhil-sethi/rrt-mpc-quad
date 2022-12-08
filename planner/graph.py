import math
import numpy as np

class Node:
    def __init__(self, pos: np.ndarray, parent=None, id=0, dist=0) -> None:
        self.id = id  # Only for debugging
        self.pos = pos
        self.parent = parent
        if parent is None:
            self.connections = [self]
            self.cost_to_come = 0. # for RRT*
        else:
            self.connections = self.parent.connections + [self]
            self.cost_to_come = self.parent.cost_to_come + dist

    def add_cost(self,distance) -> None:
        self.cost_to_come = self.cost_to_come + distance

class Graph:
    def __init__(self, init_node = None) -> None:
        self.nodes = []
        if init_node != None:
            self.nodes.append(init_node)

    def add_node(self, point):
        self.nodes.append(point)

    def remove_node(self, node: Node):
        id = node.id
        for i in range(len(self.graph.nodes)):
            if self.graph.nodes[i].id == id:
                self.graph.nodes.remove(i)
        return


    @staticmethod
    def euclidean_metric(a, b):
        """Euclidean distance"""
        return math.sqrt(sum([(a[i] - b[i])**2 for i in range(len(a))]))

    def closest_node(self, point) -> Node:
        min_dist = math.inf
        closest_node = None
        for node in self.nodes:
            dist = self.euclidean_metric(node.pos, point) 
            if  dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node
        
    def cost_to_come_nodes(self, point:list) -> list:

        sorted_nodes = sorted(self.nodes, key=lambda n: n.cost_to_come + self.euclidean_metric(n.pos,point))[:max(len(self.nodes), 20)] # sort based on cost-to-come to new poit
        
        return sorted_nodes