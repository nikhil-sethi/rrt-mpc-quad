import numpy as np
import matplotlib.pyplot as plt

MAX_ITER = 1000
DIST_TH = 0.01
PERC_2_END_GOAL = 0.01 # This is the percentage of evaluations at the goal position

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
    def __init__(self, start_node: Node):
        self.nodes = [start_node]
    
    def add_node(self, node: Node):
        self.nodes.append(node)

class Workspace:
    def __init__(self, bounds_x: np.ndarray, bounds_y: np.ndarray, bounds_z: np.ndarray):
        self.bounds_x = bounds_x
        self.bounds_y = bounds_y
        self.bounds_z = bounds_z   
        # Add the obstacles in this class    

class RRT:
    def __init__(self, start_node: Node, end_node: Node, workspace: Workspace):
        self.graph = Graph(start_node)
        self.end_goal = end_node
        self.ws = workspace
        self.reached_goal = False
        self.perc_endgoal = PERC_2_END_GOAL
        self.nr_nodes = 1
    
    def sample_node_position(self) -> np.ndarray:
        x_sample = np.array(list(np.random.uniform(self.ws.bounds_x[0],self.ws.bounds_x[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[0]])
        y_sample = np.array(list(np.random.uniform(self.ws.bounds_y[0],self.ws.bounds_y[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[1]])
        z_sample =  np.array(list(np.random.uniform(self.ws.bounds_z[0], self.ws.bounds_z[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[2]])
        samples = np.vstack((x_sample, y_sample, z_sample)).T
        chosen_sample = samples[np.random.choice(len(samples))]
        return chosen_sample
    
    def check_collision_node(self, node_pos: np.ndarray) -> bool:
        return False

    def find_closest_node(self, sample_point: np.ndarray) -> bool:
        closest_node = self.graph.nodes[np.argmin(np.linalg.norm(sample_point - np.array([self.graph.nodes[i].pos for i in range(len(self.graph.nodes))]), axis=1))]
        return closest_node

    def check_collision_connection(self, node_a_pos: np.ndarray, node_b_pos: np.ndarray) -> bool:
        return False
    
    def check_reached_endgoal(self):
        self.reached_goal = (np.linalg.norm((self.graph.nodes[-1].pos - self.end_goal.pos)) < DIST_TH)

    def plan(self):
        new_node_pos = self.sample_node_position()
        collision_node = self.check_collision_node(new_node_pos)
        if collision_node:
            return
        closest_node = self.find_closest_node(new_node_pos)
        collision_connection = self.check_collision_connection(closest_node.pos, new_node_pos)
        if collision_connection:
            return
        new_node = Node(pos=new_node_pos, parent=closest_node, id=self.nr_nodes)
        self.nr_nodes+=1
        self.graph.add_node(new_node)
        self.check_reached_endgoal()

def plot_final_path(nodes: list, start_node: Node, end_node: Node):
    plt.figure()
    plt.title("final graph")
    for j in range(len(nodes)):
        path_x = np.array([nodes[j].connections[i].pos[0] for i in range(len(nodes[j].connections))])
        path_y = np.array([nodes[j].connections[i].pos[1] for i in range(len(nodes[j].connections))])
        plt.plot(path_x, path_y,"r--")
    plt.plot(path_x, path_y,"g")  
    plt.plot(start_node.pos[0], start_node.pos[1], "bo", markersize=10, label="start")
    plt.plot(end_node.pos[0], end_node.pos[1], "go", markersize=10, label="goal")
    plt.legend()
    plt.grid()
    plt.show()


def main():
    start_node = Node(pos=np.array([1,1,0]))
    end_node = Node(pos=np.array([5,5,0]), id=-1)
    WS = Workspace(np.array([0,5]), np.array([0,5]), np.array([0,0]))
    rrt_planner = RRT(start_node=start_node, end_node=end_node, workspace=WS)
    iter=0
    while (not(rrt_planner.reached_goal) and (iter<MAX_ITER)):
        rrt_planner.plan()
        iter+=1

    plot_final_path(rrt_planner.graph.nodes, start_node, end_node)


if __name__ == "__main__":
    main()
