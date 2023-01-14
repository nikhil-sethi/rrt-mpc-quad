import math
import numpy as np
from utils.color import Color

class Node:
     def __init__(self, pos: list, parent=None, id=0):
        self.id = id  # Only for debugging
        self.pos = np.array(pos)
        self.parent = parent
        self.children = []
        if parent is None:
            self.connections = [self]
            self.dist_from_start = 0
        else:
            # self.parent.children.append(self)
            self.connections = self.parent.connections + [self]
            self.dist_from_start = self.parent.dist_from_start + np.linalg.norm(self.pos - self.parent.pos)

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
        parent.children.append(new_node)
        self.nodes.append(new_node)
        # print(f"Adding node id {new_node.id}")
        return new_node
    
    def remove_node(self, node: Node, env, final_node):
        # return
        # if node.id == 0:
        #     print("also wtf")
        if not self.nodes:
            return
        # if len(self.nodes)==1:
            # print("wtf")
            # print(f"Nodes in final node connections:")
            # for node in final_node.connections:
            #     print(node.id)
            # return
        if node.children:
            # print(f"Want to remove node {node.id}, but need to remove the following {len(node.children)} children first.")
            # for i in range(len(node.children)):
                # print(node.children[i].id)
            for i in range(len(node.children)):
                self.remove_node(node.children[i], env, final_node)
        if not self.nodes:
            return
        # print(f"Removing node {node.id}. Current graph length = {len(self.nodes)}")
        env.remove_line(node.id)
        nodes_arr = np.array(self.nodes.copy())
        nodes_arr = np.delete(nodes_arr, np.sum((np.array([node_s.id for node_s in nodes_arr])==node.id)*np.arange(len(nodes_arr))))
        self.nodes = list(nodes_arr)
        # print(f"After removal, graph length = {len(self.nodes)}")
        # self.nodes.pop(node.id)
  