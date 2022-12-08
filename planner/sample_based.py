from .spaces import Space#, Workspace
from .graph import Graph, Node
from utils import Quadrotor
import matplotlib.pyplot as plt
import numpy as np

MODE = 'star' # 'regular' rrt or rrt 'star'
MAX_ITER = 500
MAX_IMPR = 2 # when node is within this distance to the goal, it is connected to the goal
PERC_2_END_GOAL = 0.05 # This is the percentage of evaluations at the goal position

class SamplingPlanner:
    def __init__(self, start:Node, goal:Node, space:Space, map) -> None:
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
    
    def get_free_sample(self): # still unused
        colliding = True
        while colliding:
            sample = self.space.sample(self.goal.pos,PERC_2_END_GOAL)
            for obs in self.map:
                if obs.is_colliding(sample):
                    colliding = True
                else:
                    colliding = False
        return sample
    
    def get_free_connection(self, q, q_p): # still unused
        return (q,q_p)

class RRT(SamplingPlanner):
    def __init__(self, start:Node, goal:Node, space:Space, map) -> None:
        super().__init__(start, goal, space, map)
        self.goal = goal
        self.best_cost = np.Inf
        self.max_depth = 10
        self.reached_goal = False
        self.path_improvements = 0
        self.done = False
        self.num_nodes = len(self.graph.nodes)

    def check_reached_goal(self) -> None:
        if self.graph.nodes[-1] == self.goal:
            if self.reached_goal:
                if self.graph.nodes[-1].cost_to_come < self.best_cost:
                    self.best_cost = self.graph.nodes[-1].cost_to_come
                    self.path_improvements+=1
                    print("Current best path = ",self.best_cost)
                    if self.path_improvents > MAX_IMPR:
                        self.done = True
                return
            else:
                self.best_cost = self.graph.nodes[-1].cost_to_come
                self.reached_goal = True
                return

    def garbage_collection(self):
        if self.reached_goal:
            for node in self.graph.nodes:
                if node.cost_to_come + self.graph.euclidean_metric(node.pos,self.goal.pos) > self.best_cost:
                    self.graph.remove_node(node)
        elif self.graph.nodes[-1].cost_to_come + self.graph.euclidean_metric(self.graph.nodes[-1].pos,self.goal.pos) > self.best_cost:
            self.graph.remove_node(self.graph.nodes[-1])
        print("Collecting garbage... ",self.num_nodes," nodes remaining.")

    def result(self) -> None:
        print("Final path distance after ",self.path_improvements," improvements is ",round(self.graph.nodes[-1].cost_to_come,3))
        
    
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

    if MODE == 'regular':
        def step(self):
            """one step of the planner"""
            new_node_pos = self.space.sample(self.goal.pos,PERC_2_END_GOAL)
            if self.check_collision_point(new_node_pos):
                return

            closest_node = self.graph.closest_node(new_node_pos) # of type Node
            if self.check_collision_connection(closest_node.pos, new_node_pos):
                return
            

            new_node = Node(pos=new_node_pos, parent=closest_node, id=self.num_nodes)
            self.num_nodes+=1
            self.graph.add_node(new_node)
            self.check_reached_goal()
    elif MODE == 'star':
        def step(self):
            """one step of the planner"""
            new_node_pos = self.space.sample(self.goal.pos,PERC_2_END_GOAL)
            if self.check_collision_point(new_node_pos):
                return

            cost_to_come_list = self.graph.cost_to_come_nodes(new_node_pos)
            for node in cost_to_come_list:
                coll = self.check_collision_connection(node.pos,new_node_pos)
                if not coll:
                    parent_node = node
                    break
            
            if coll:    # if no connection is collision free
                return

            dist = self.graph.euclidean_metric(new_node_pos,parent_node.pos)
            new_node = Node(pos=new_node_pos, parent=parent_node, id=self.num_nodes, dist=dist)
            self.num_nodes+=1
            self.graph.add_node(new_node)
            self.check_reached_goal()
            self.garbage_collection()
    else:
        raise NotImplementedError()


    def backtrack(self, point):
        """Returns a continuous connected path from point to start node"""
        pass

    def run(self):
        for i in range(MAX_ITER):
            self.step()
            i+=1
            if self.done:
                break
        if self.reached_goal:
            self.result()

        return self.graph.nodes[-1].connections


# class RRT_Star(SamplingPlanner):
#     def __init__(self, start: Node, goal: Node, space:Space, map:list):
#         self.graph = Graph(start_node)
#         self.start_node = start_node
#         self.end_goal = end_node
#         self.ws = workspace
#         self.reached_goal = False
#         self.perc_endgoal = PERC_2_END_GOAL
#         self.nr_nodes = 1
#         self.drone = drone
#         self.fastest_route_to_end = np.inf
#         self.final_node = None

#     def check_reached_goal(self):
#         dist = self.graph.euclidean_metric(self.graph.nodes[-1].pos, self.goal.pos)
#         if dist < DIST_TH:
#             self.reached_goal = True
    
#     def sample_node_position(self) -> np.ndarray:
#         x_sample = np.array(list(np.random.uniform(self.ws.bounds_x[0],self.ws.bounds_x[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[0]])
#         y_sample = np.array(list(np.random.uniform(self.ws.bounds_y[0],self.ws.bounds_y[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[1]])
#         z_sample =  np.array(list(np.random.uniform(self.ws.bounds_z[0], self.ws.bounds_z[1], int(1//self.perc_endgoal))) + [self.end_goal.pos[2]])
#         samples = np.vstack((x_sample, y_sample, z_sample)).T
#         chosen_sample = samples[np.random.choice(len(samples))]
#         return chosen_sample
    
#     def check_collision_node(self, node_pos: np.ndarray, obstacles: list, drone: Quadrotor) -> bool:
#         for obs in obstacles:
#             coll = obs.check_collision(drone, node_pos)
#             if coll:
#                 return True
#         return False

#     def find_closest_node(self, sample_point: np.ndarray):
#         closest_node = self.graph.nodes[np.argmin(np.linalg.norm(sample_point - np.array([self.graph.nodes[i].pos for i in range(len(self.graph.nodes))]), axis=1))]
#         return closest_node
    
#     def find_lowest_cost_node(self, sample_point: np.ndarray):
#         close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - sample_point))[:max(len(self.graph.nodes), 20)]
#         max_dist = np.inf
#         shortest_path_node = None
#         for node in close_nodes:
#             collision_connection = self.check_collision_connection(node.pos, sample_point, self.ws.obstacles, self.drone)
#             if collision_connection:
#                 continue
#             dist = node.dist_from_start + np.linalg.norm(node.pos - sample_point)
#             if dist < max_dist:
#                 shortest_path_node = node
#                 max_dist = dist

#         return shortest_path_node


#     def check_collision_connection(self, node_a_pos: np.ndarray, node_b_pos: np.ndarray, obstacles: list, drone: Quadrotor) -> bool:
#         N = 20
#         x_s = np.linspace(node_a_pos[0], node_b_pos[0], N)
#         y_s = np.linspace(node_a_pos[1], node_b_pos[1], N)
#         z_s = np.linspace(node_a_pos[2], node_b_pos[2], N)
#         sample_node_pos = np.vstack((x_s, y_s, z_s)).T
#         for i in range(sample_node_pos.shape[0]):
#             coll = self.check_collision_node(sample_node_pos[i], obstacles, drone)
#             if coll:
#                 return True
#         return False
    

#     def plan(self):
#         new_node_pos = self.sample_node_position()
#         collision_node = self.check_collision_node(new_node_pos, self.ws.obstacles, self.drone)
#         if collision_node:
#             return

#         closest_node = self.find_lowest_cost_node(new_node_pos)
#         if closest_node is None:
#             return
        
#         new_node = Node(pos=new_node_pos, parent=closest_node, id=self.nr_nodes)

#         self.graph.add_node(new_node)
#         self.nr_nodes+=1
#         self.check_shortcut_for_nodes(new_node)
#         if (np.linalg.norm(self.graph.nodes[-1].pos -self.end_goal.pos)<DIST_TH) and (self.graph.nodes[-1].dist_from_start < self.fastest_route_to_end):
#             self.fastest_route_to_end = self.graph.nodes[-1].dist_from_start
#             self.final_node = self.graph.nodes[-1]
#             self.reached_goal = True
        
#         self.garbage_collection()

#     def garbage_collection(self):
#         if np.linalg.norm(self.graph.nodes[-1].pos -self.end_goal.pos)<DIST_TH:
#             for node in self.graph.nodes:
#                 if node.dist_from_start + np.linalg.norm(self.end_goal.pos - node.pos) > self.fastest_route_to_end:
#                     self.graph.remove_node(node)
#         elif self.graph.nodes[-1].dist_from_start + np.linalg.norm(self.end_goal.pos - self.graph.nodes[-1].pos) > self.fastest_route_to_end:
#             self.graph.remove_node(self.graph.nodes[-1])

        

#     def reroute(self, node_s, new_node):
#         collision_connection = self.check_collision_connection(node_s.pos, new_node.pos, self.ws.obstacles, self.drone)
#         if collision_connection:
#             return
#         rerouted_node = Node(pos=node_s.pos, parent=new_node, id=self.nr_nodes)
#         self.graph.add_node(rerouted_node)

#         self.nr_nodes+=1
#         self.graph.remove_node(node_s)
        

#     def check_shortcut_for_nodes(self, new_node):
#         close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - new_node.pos))[1:max(len(self.graph.nodes), 10)]
#         for node in close_nodes:
#             shortcut_bool = new_node.dist_from_start + np.linalg.norm(node.pos - new_node.pos) < node.dist_from_start - 0.0000001
#             if shortcut_bool:
#                 self.reroute(node, new_node)