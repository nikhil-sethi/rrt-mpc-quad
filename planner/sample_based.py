from .spaces import Space
from .graph import Graph, Node
import matplotlib.pyplot as plt
import numpy as np

DIST_TH = 0.2

class SamplingPlanner:
    def __init__(self, start, goal, space:Space, map) -> None:
        self.start = start
        self.goal = goal
        self.map:list = map # list of obstacles
        self.space = space
        self.graph = Graph(init_node=start) 
    
    def check_collision_connection(self, node_a_pos:list, node_b_pos:list):
        N = 20
        x_s = np.linspace(node_a_pos[0], node_b_pos[0], N)
        y_s = np.linspace(node_a_pos[1], node_b_pos[1], N)
        z_s = np.linspace(node_a_pos[2], node_b_pos[2], N)
        for i in range(x_s.shape[0]):
            coll = self.check_collision_point([x_s[i], y_s[i], z_s[i]])
            if coll:
                return True
        return False

    def check_collision_point(self, point:list):
        for obs in self.map:
            if obs.is_colliding(point):
                return True
        return False
    
    def get_free_sample(self):
        colliding = True
        while colliding:
            sample = self.space.sample()
            for obs in self.map:
                if obs.is_colliding(sample):
                    colliding = True
                else:
                    colliding = False
        return sample
    
    def get_free_connection(self, q, q_p):
        return (q,q_p)

class RRT(SamplingPlanner):
    def __init__(self, start, goal, space:Space, map) -> None:
        super().__init__(start, goal, space, map)
        self.best_cost = 0
        self.max_depth = 10
        self.reached_goal = False
        self.num_nodes = len(self.graph.nodes)

    def check_reached_goal(self):
        dist = self.graph.euclidean_metric(self.graph.nodes[-1].pos, self.goal.pos)
        print(round(dist,3))
        if dist < DIST_TH:
            self.reached_goal = True
            print("Final distance",dist)
    
    def plot_final_path(self):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        plt.title("final graph")
        nodes = self.graph.nodes

        for j in range(len(nodes)):
            path_x = np.array([nodes[j].connections[i].pos[0] for i in range(len(nodes[j].connections))])
            path_y = np.array([nodes[j].connections[i].pos[1] for i in range(len(nodes[j].connections))])
            path_z = np.array([nodes[j].connections[i].pos[2] for i in range(len(nodes[j].connections))])
            ax.plot3D(path_x, path_y, path_z,"r--")
        ax.plot3D(path_x, path_y, path_z, "g")  
        ax.plot3D(self.start.pos[0], self.start.pos[1],self.start.pos[2], "bo", markersize=10, label="start")
        ax.plot3D(self.goal.pos[0], self.goal.pos[1],self.start.pos[2], "go", markersize=10, label="goal")
        plt.legend()
        plt.grid()
        plt.show()

    def step(self):
        """one step of the planner"""
        new_node_pos = self.space.sample()
        if self.check_collision_point(new_node_pos):
            return

        closest_node = self.graph.closest_node(new_node_pos) # of type Node
        if self.check_collision_connection(closest_node.pos, new_node_pos):
            return

        new_node = Node(pos=new_node_pos, parent=closest_node, id=self.num_nodes)
        self.num_nodes+=1
        self.graph.add_node(new_node)
        self.check_reached_goal()

    def backtrack(self, point):
        """Returns a continuous connected path from point to start node"""
        pass

    def run(self):
        # for i in range(self.max_iters):
        while not self.reached_goal:
            self.step()

        return self.graph.nodes[-1].connections

