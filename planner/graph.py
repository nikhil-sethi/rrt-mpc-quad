import math
import numpy as np
from utils.color import Color

class Node:
     def __init__(self, pos: list, parent=None, id: int=0):
        self.id = id
        self.pos = np.array(pos)
        self.parent = parent # the node to which this node is connected
        self.children = []  # Nodes that are connected to this node
        if parent is None:
            self.connections = [self]
            self.dist_from_start = 0
        else:
            parent.children += [self]
            self.connections = self.parent.connections + [self]
            self.dist_from_start = self.parent.dist_from_start + np.linalg.norm(self.pos - self.parent.pos)

# The Graph class consists of all the Nodes that are placed in the workspace
class Graph:
    def __init__(self, start_node: Node):
        self.nodes = [start_node]
        self.num_nodes_created = 1
        self.lines_plotted = 0

    def add_node(self, new_node_pos, parent: Node, env) -> Node:
        self.num_nodes_created += 1
        if env.plot_all:
            self.lines_plotted += 1
            new_node_id = env.plot_line(new_node_pos, parent.pos, lineWidth=2.5)
        else:
            new_node_id = self.num_nodes_created
        new_node = Node(pos=new_node_pos, parent=parent, id=new_node_id)
        self.nodes.append(new_node)
        return new_node
    
    def remove_node(self, node: Node, env, final_node):
        if not self.nodes:
            return
        if node.children:
            for i in range(len(node.children)):
                self.remove_node(node.children[i], env, final_node)  # If a node is removed, also remove the children nodes, otherwise they will be hanging
        if not self.nodes:
            return
        env.remove_line(node.id)
        nodes_arr = np.array(self.nodes.copy())
        nodes_arr = np.delete(nodes_arr, np.sum((np.array([node_s.id for node_s in nodes_arr])==node.id)*np.arange(len(nodes_arr))))
        self.nodes = list(nodes_arr)
        