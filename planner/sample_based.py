from .spaces import Space#, Workspace
from .graph import Graph, Node
import matplotlib.pyplot as plt
import numpy as np
from utils import printRed

DIST_TH = 0.01
MAX_ITER = 500
MAX_IMPR = 10 # number of improvements the rrt* algorithm makes before it stops
PERC_2_GOAL = 0.25 # This is the percentage of evaluations at the goal position

class SamplingPlanner:
    def __init__(self, start:Node, goal:Node, space:Space, map) -> None:
        self.start = start
        self.goal = goal
        self.map:list = map # list of obstacles
        self.space = space
        self.graph = Graph(start_node=start)
        self.reached_goal = False
        self.perc_goal = PERC_2_GOAL
        self.nr_nodes = 1
        self.fastest_route_to_end = np.inf
        self.final_node:Node = None
    
    def check_collision_connection(self, node_a_pos:np.ndarray, node_b_pos:np.ndarray):
        N = int(np.linalg.norm(node_a_pos - node_b_pos)/0.05) + 1
        x_s = np.linspace(node_a_pos[0], node_b_pos[0], N)
        y_s = np.linspace(node_a_pos[1], node_b_pos[1], N)
        z_s = np.linspace(node_a_pos[2], node_b_pos[2], N)
        for i in range(x_s.shape[0]):
            coll = self.check_collision_node(np.array([x_s[i], y_s[i], z_s[i]]))
            if coll:
                return True
        return False

    def check_collision_node(self, point:np.ndarray) -> bool:
        for obs in self.map:
            if obs.is_colliding(point):
                return True
        return False
    
    def sample_node_position(self) -> np.ndarray:
        x_sample = np.array(list(np.random.uniform(self.space.bounds_x[0],self.space.bounds_x[1], int(1//self.perc_goal))) + [self.goal.pos[0]])
        y_sample = np.array(list(np.random.uniform(self.space.bounds_y[0],self.space.bounds_y[1], int(1//self.perc_goal))) + [self.goal.pos[1]])
        z_sample =  np.array(list(np.random.uniform(self.space.bounds_z[0], self.space.bounds_z[1], int(1//self.perc_goal))) + [self.goal.pos[2]])
        samples = np.vstack((x_sample, y_sample, z_sample)).T
        chosen_sample = samples[np.random.choice(len(samples))]
        return chosen_sample
    
    def check_reached_goal(self):
        self.reached_goal = (np.linalg.norm((self.graph.nodes[-1].pos - self.goal.pos)) < DIST_TH)
    
    def plan(self):
        raise NotImplementedError()

    @staticmethod
    def nodes_to_array(nodes):
        """Converts list of nodes to array of waypoints"""
        return np.array([node.pos for node in nodes])

    def run(self) -> list:
        for i in range(MAX_ITER):
            self.plan()
            # print("dfg")
            
        if self.reached_goal:
            printRed(f"[Planner] Goal Reached! Total distance: {self.final_node.dist_from_start}")
            self.fastest_route_to_end = self.final_node.dist_from_start
            return self.final_node.connections
        else:
            printRed("[Planner] Goal not reached")
            printRed(f"Number of nodes: {self.nr_nodes}")
            # assert(self.reached_goal)
            return self.graph.nodes[-1].connections

    @staticmethod
    def discretize_path(connections, num_steps=200) -> np.ndarray:
        # each node in connections has a position (list of x,y,z)
        # need to define a line/vector from parent pos to node position
        traj = np.empty((0,3))
        for i in range(len((connections))-1):
            node_start = connections[i] 
            node_end = connections[i+1]  
            traj_btw_nodes = np.linspace(node_start, node_end, num=num_steps, endpoint=False)
            traj = np.append(traj, traj_btw_nodes, axis=0)
        traj = np.append(traj, connections[-1][None,:], axis=0)
        return traj

class RRT(SamplingPlanner):
    def __init__(self,start:Node, goal:Node, space:Space, map):
        super().__init__(start, goal, space, map)

    def find_closest_node(self, sample_point: np.ndarray) -> bool:
        closest_node = self.graph.nodes[np.argmin(np.linalg.norm(sample_point - np.array([self.graph.nodes[i].pos for i in range(len(self.graph.nodes))]), axis=1))]
        return closest_node

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
        if self.nr_nodes == 1: self.final_node = self.graph.nodes[-1]
        self.check_reached_goal()
        if self.final_node != None:
            if self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
                self.final_node = self.graph.nodes[-1]

    # def run(self) -> list:
    #     for i in range(MAX_ITER):
    #         self.plan()
    #         # print("dfg")
            
    #         if self.reached_goal:
    #             printRed(f"[Planner] Goal Reached! Total distance: {self.final_node.dist_from_start}")
    #             self.fastest_route_to_end = self.final_node.dist_from_start
    #             return self.final_node.connections
    #     else:
    #         assert(self.reached_goal, "\033[91m [Planner] Goal not reached \033[00m")
    #         return self.graph.nodes[-1].connections



class RRT_Star(SamplingPlanner):
    def __init__(self,start:Node, goal:Node, space:Space, map):
        super().__init__(start, goal, space, map)
    
    def find_lowest_cost_node(self, sample_point: np.ndarray):
        close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - sample_point))[:max(len(self.graph.nodes), 20)]
        max_dist = np.inf
        shortest_path_node = None
        for node in close_nodes:
            collision_connection = self.check_collision_connection(node.pos, sample_point)
            if collision_connection:
                continue
            dist = node.dist_from_start + np.linalg.norm(node.pos - sample_point)
            if dist < max_dist:
                shortest_path_node = node
                max_dist = dist

        return shortest_path_node
    
    def plan(self):
        new_node_pos = self.sample_node_position()
        collision_node = self.check_collision_node(new_node_pos)
        if collision_node:
            return

        closest_node = self.find_lowest_cost_node(new_node_pos)
        if closest_node is None:
            return
        
        new_node = Node(pos=new_node_pos, parent=closest_node, id=self.nr_nodes)

        self.graph.add_node(new_node)
        self.nr_nodes+=1
        self.check_shortcut_for_nodes(new_node)
        if (np.linalg.norm(self.graph.nodes[-1].pos -self.goal.pos)<DIST_TH) and (self.graph.nodes[-1].dist_from_start < self.fastest_route_to_end):
            self.final_node = self.graph.nodes[-1]
            self.fastest_route_to_end = self.graph.nodes[-1].dist_from_start
            self.reached_goal = True
        
        self.garbage_collection()

    def garbage_collection(self):
        if np.linalg.norm(self.graph.nodes[-1].pos -self.goal.pos)<DIST_TH:
            for node in self.graph.nodes:
                if node.dist_from_start + np.linalg.norm(self.goal.pos - node.pos) > self.fastest_route_to_end:
                    self.graph.remove_node(node)
        elif self.graph.nodes[-1].dist_from_start + np.linalg.norm(self.goal.pos - self.graph.nodes[-1].pos) > self.fastest_route_to_end:
            self.graph.remove_node(self.graph.nodes[-1])

        

    def reroute(self, node_s, new_node):
        collision_connection = self.check_collision_connection(node_s.pos, new_node.pos)
        if collision_connection:
            return
        rerouted_node = Node(pos=node_s.pos, parent=new_node, id=self.nr_nodes)
        self.graph.add_node(rerouted_node)

        self.nr_nodes+=1
        self.graph.remove_node(node_s)
        

    def check_shortcut_for_nodes(self, new_node):
        close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - new_node.pos))[1:max(len(self.graph.nodes), 10)]
        for node in close_nodes:
            shortcut_bool = new_node.dist_from_start + np.linalg.norm(node.pos - new_node.pos) < node.dist_from_start - 0.0000001
            if shortcut_bool:
                self.reroute(node, new_node)
