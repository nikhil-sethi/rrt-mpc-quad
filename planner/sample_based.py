from .spaces import Space
from .graph import Graph, Node
import matplotlib.pyplot as plt
import numpy as np
# from environment import Env
from utils import printRed

DIST_TH = 0.01
MAX_ITER = 500
MAX_IMPR = 10 # number of improvements the rrt* algorithm makes before it stops
PERC_2_GOAL = 0.1 # This is the percentage of evaluations at the goal position

class SamplingPlanner:
    def __init__(self, start:Node, goal:Node, space:Space, map, env, result:dict = {}) -> None:
        self.start = start
        self.goal = goal
        self.map:list = map # list of obstacles
        self.combined_T_inv = np.array([obs.T_inv for obs in map])
        self.combined_bbox = np.array([obs.bbox_arr for obs in map])
        self.space = space
        self.graph = Graph(start_node=start)
        self.reached_goal = False
        self.perc_goal = PERC_2_GOAL
        self.nr_nodes = 1
        self.nr_nodes_gc = 1
        self.fastest_route_to_end = np.inf
        self.final_node:Node = None
        self.result = result
        self.env = env
        self.transformed_point = np.ones((4,1))
    
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
        # ret = False
        # for obs in self.map:
        #     if obs.is_colliding(point):
        #         ret = True
        #         break
        ret = self.are_colliding(point)
        return ret
        # return False
        # print("2", False)
        # return self.are_colliding(point)

    def are_colliding(self, point):
        self.transformed_point[0:3] = point[:, None]
        self.transformed_point[3] = 1
        transformed_points = self.combined_T_inv @ self.transformed_point
        ret = any(np.all(((-self.combined_bbox/2 < transformed_points[:,:3]) * (self.combined_bbox/2 > transformed_points[:,:3])), axis=1))
        return ret

    def sample_node_position(self) -> np.ndarray:
        x_sample = np.array(list(np.random.uniform(self.space.bounds_x[0],self.space.bounds_x[1], int(1//self.perc_goal))) + [self.goal.pos[0]])
        y_sample = np.array(list(np.random.uniform(self.space.bounds_y[0],self.space.bounds_y[1], int(1//self.perc_goal))) + [self.goal.pos[1]])
        z_sample = np.array(list(np.random.uniform(self.space.bounds_z[0], self.space.bounds_z[1], int(1//self.perc_goal))) + [self.goal.pos[2]])
        samples = np.vstack((x_sample, y_sample, z_sample)).T
        chosen_sample = samples[np.random.choice(len(samples))]
        return chosen_sample
    
    def check_reached_goal(self):
        # Function only used by RRT planner
        # Separate booleans are tracked for graph and last node
        self.last_node_in_goal = (np.linalg.norm((self.graph.nodes[-1].pos - self.goal.pos)) < DIST_TH)
        if not self.reached_goal and self.last_node_in_goal:
            self.reached_goal = True
    
    def plan(self):
        raise NotImplementedError()

    @staticmethod
    def nodes_to_array(nodes):
        """Converts list of nodes to array of waypoints"""
        return np.array([node.pos for node in nodes])

    def run(self) -> list:
        for i in range(MAX_ITER):
            self.plan()

        self.plot_all_nodes()
        assert (self.reached_goal == True), "\033[91m [Planner] Goal not reached \033[00m"

        printRed(f"[Planner] Goal Reached! Total distance: {self.final_node.dist_from_start}")

        # compile results/metrics
        self.result["global_planner"]["metrics"]["path_length"] = self.fastest_route_to_end
        self.result["global_planner"]["metrics"]["nodes_wo_gc"] = self.nr_nodes
        self.result["global_planner"]["metrics"]["nodes_w_gc"] = self.nr_nodes_gc

        self.fastest_route_to_end = self.final_node.dist_from_start
        return self.final_node.connections
    
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

    def plot_all_nodes(self):
        # Print all nodes
        for end_node in self.graph.nodes:
            self.env.plot_point(end_node.pos)
            if end_node.parent is not None:
                self.env.plot_line(end_node.parent.pos, end_node.pos)
            # connections = end_node.connections
            # prev_pos = env.INIT_XYZS[0]		
            # for node in connections:
            #     env.plot_point(node.pos)
            #     # try:
            #     env.plot_line(prev_pos, node.pos)
            #     prev_pos = node.pos

    def constrict_WS(self):
        dil = 0
        ws_dilation = np.array([dil, dil, dil])
        all_limits = []
        for node in self.final_node.connections:
            all_limits.append(np.abs(node.pos))
        all_limits.append(self.start.pos)
        all_limits.append(self.goal.pos)
        all_limits = np.array(all_limits)
        new_ul = np.max(all_limits, axis = 0) + ws_dilation
        new_ll = np.min(all_limits, axis = 0) - ws_dilation
        if new_ll[2] <= 0: new_ll[2] = 0
        self.space = Space(low = new_ll, high = new_ul)
        printRed(f"Constricting. New UL: {self.space.hl}, New LL: {self.space.ll}")

class RRT(SamplingPlanner):
    def __init__(self,start:Node, goal:Node, space:Space, map, env, result:dict = {}):
        super().__init__(start, goal, space, map, env, result)

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
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = self.graph.nodes[-1]
            elif self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
                self.final_node = self.graph.nodes[-1]

class Informed_RRT(RRT):
    def __init__(self, start:Node, goal:Node, space:Space, map, env, result:dict = {}):
        super().__init__(start, goal, space, map, env, result)
        printRed(f"Starting WS: UL: {self.space.hl}, LL: {self.space.ll}")

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
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = self.graph.nodes[-1]
                self.constrict_WS()
                printRed(self.graph.nodes[-1].dist_from_start)
            elif self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
                self.final_node = self.graph.nodes[-1]
                self.constrict_WS()
                printRed(self.graph.nodes[-1].dist_from_start)

class Recycle_RRT(RRT):
    def __init__(self,start:Node, goal:Node, space:Space, map, env, result:dict = {}):
        super().__init__(start, goal, space, map, env, result)

    def clear_unused_nodes(self):
        used_idxs = []
        printRed(f"Initial Number of Nodes {self.nr_nodes}")
        for node in self.final_node.connections:
            used_idxs.append(self.graph.nodes.index(node))
        for i in reversed(range(len(self.graph.nodes))):
            if i not in used_idxs:
                self.graph.nodes.pop(i)
        self.nr_nodes = len(self.graph.nodes)
        printRed(f"Reduced Number of Nodes {self.nr_nodes}")

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
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = self.graph.nodes[-1]
            elif self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
                self.final_node = self.graph.nodes[-1]
                self.clear_unused_nodes()

class RRT_Star(SamplingPlanner):
    def __init__(self, start: Node, goal: Node, space: Space, map, env, result: dict = {}):
        super().__init__(start, goal, space, map, env, result)
    
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
                    self.nr_nodes_gc += 1
        elif self.graph.nodes[-1].dist_from_start + np.linalg.norm(self.goal.pos - self.graph.nodes[-1].pos) > self.fastest_route_to_end:
            self.graph.remove_node(self.graph.nodes[-1])
            self.nr_nodes_gc += 1

    def reroute(self, node_s, new_node):
        collision_connection = self.check_collision_connection(node_s.pos, new_node.pos)
        if collision_connection:
            return
        rerouted_node = Node(pos=node_s.pos, parent=new_node, id=self.nr_nodes)
        self.graph.add_node(rerouted_node)

        self.nr_nodes += 1
        self.graph.remove_node(node_s)
 
    def check_shortcut_for_nodes(self, new_node):
        close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - new_node.pos))[1:max(len(self.graph.nodes), 10)]
        for node in close_nodes:
            shortcut_bool = new_node.dist_from_start + np.linalg.norm(node.pos - new_node.pos) < node.dist_from_start - 0.0000001
            if shortcut_bool:
                self.reroute(node, new_node)

class Informed_RRT_Star(RRT_Star):
    def __init__(self,start: Node, goal: Node, space: Space, map, env, result: dict = {}):
        super().__init__(start, goal, space, map, env, result)
        printRed(f"Starting WS: UL: {self.space.hl}, LL: {self.space.ll}")

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
            self.constrict_WS()
        
        self.garbage_collection()
