import math

class Node:
    pass

class Graph:
    def __init__(self, init_node = None) -> None:
        self.nodes = []
        if init_node != None:
            self.nodes.append(init_node)

    def add_node(self, point):
        self.nodes.append(point)

    def add_edge(self, e):
        self.edges.append(e)



    @staticmethod
    def euclidean_metric(a, b):
        """Euclidean distance"""
        return math.sqrt(sum([(a[i]-b[i])**2 for i in range(3)]))

    def closest_node(self, point):
        min_dist = math.inf
        closest_node = None
        for node in self.nodes:
            dist = self.euclidean_metric(node, point) 
            if  dist < min_dist:
                min_dist = dist
                closest_node = node
        return closest_node, min_dist