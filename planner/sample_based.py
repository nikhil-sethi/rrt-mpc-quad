from .spaces import Space
from .graph import Graph, Node
from utils.color import Color
import matplotlib.pyplot as plt
import numpy as np
from utils import printC
from utils.color import PrintColor

DIST_TH = 0.01
MIN_ITER = [200, 200, 200, 1300, 200, 200, 200, 500]
MAX_ITER = [3000, 3000, 3000, 5000, 3000, 3000, 3000, 3000]
MAX_IMPR = 10 # number of improvements the rrt* algorithm makes before it stops
PERC_2_GOAL = 0.05 # This is the percentage of evaluations at the goal position

class SamplingPlanner:
    def __init__(self, start:Node, goal:Node, space:Space, map, env, result:dict = {}, map_number:int=0) -> None:
        self.start = start
        self.goal = goal
        self.map:list = map # list of obstacles
        self.combined_T_inv = np.array([obs.T_inv for obs in map])
        self.combined_bbox = np.array([obs.bbox_arr for obs in map])
        self.space = space
        self.graph = Graph(start_node=start)
        self.reached_goal = False
        self.perc_goal = PERC_2_GOAL
        # self.nr_nodes = 1
        self.nr_nodes_gc = 1
        self.fastest_route_to_end = np.inf
        self.final_node:Node = None
        self.result = result
        self.env = env
        self.transformed_point = np.ones((4,1))
        self.min_iter = MIN_ITER[map_number]
        self.max_iter = MAX_ITER[map_number]
        self.num_impr = 0
        self.plot_all = env.plot_all
        self.name = None

    def run(self) -> list:
        iter_num = 0
        for i in range(self.min_iter):
            self.plan()
            iter_num += 1
            # if self.reached_goal and (self.name == 'rrt' or self.name == 'rec_rrt' or self.name == 'inf_rrt'):
            #     print("test")
            #     break
        while iter_num < self.max_iter and not self.reached_goal:
            self.plan()
            iter_num += 1

        print(f"[Informed RRT] Completed iterations: {iter_num} of maximum alloted {self.max_iter}")
        self.result["global_planner"]["metrics"]["iter_num"] = iter_num

        assert (self.reached_goal == True), "\033[91m [Planner] Goal not reached \033[00m"

        printC(f"[Planner] Goal Reached! Total distance: {self.final_node.dist_from_start}")
        self.result['text_output'] += f"[Planner] Goal Reached! Total distance: {self.final_node.dist_from_start}\n"

        # compile results/metrics
        # self.result["global_planner"]["metrics"]["path_length"] = self.fastest_route_to_end
        self.result["global_planner"]["metrics"]["nodes_wo_gc"] = self.graph.lines_plotted
        self.result["global_planner"]["metrics"]["nodes_w_gc"] = self.nr_nodes_gc

        self.fastest_route_to_end = self.final_node.dist_from_start

        if self.plot_all:
            num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.RED)
            self.graph.lines_plotted += num_lines_formed

        if self.plot_all: self.plot_text()
        # Clear away debug text, points, and lines
        for node_id in range(self.graph.lines_plotted + self.nr_nodes_gc + 1):
            self.env.remove_line(node_id)
        if self.plot_all: self.plot_text()
        
        return self.final_node.connections
    
    def plot_text(self):
        textPosition = [-0.5,0.5,1.7]
        textSize = 2
        if self.name == 'rrt':
            self.env.add_text(text="RRT PLANNER", textPosition=textPosition, textColorRGB=[1,1,1], textSize=textSize)
        elif self.name == 'inf_rrt':
            self.env.add_text(text="INFORMED RRT PLANNER", textPosition=textPosition, textColorRGB=[1,1,1], textSize=textSize)
        elif self.name == 'rec_rrt':
            self.env.add_text(text="RECYCLE RRT PLANNER", textPosition=textPosition, textColorRGB=[1,1,1], textSize=textSize)
        elif self.name == 'rrt_star':
            self.env.add_text(text="RRT STAR PLANNER", textPosition=textPosition, textColorRGB=[1,1,1], textSize=textSize)
        elif self.name == 'inf_rrt_star':
            self.env.add_text(text="INFORMED RRT STAR PLANNER", textPosition=textPosition, textColorRGB=[1,1,1], textSize=textSize)
    
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
        ret = self.are_colliding(point)
        return ret

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
        raise NotImplementedError

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
        # printC(f"[Informed RRT] Constricting. New UL: {self.space.hl}, New LL: {self.space.ll}")
    
    def execute_path_hunt(self, hunter_node: Node):
        # Setup routine variables
        created_dots = []
        impr_th = 0.1 # How much need the path be improved to bother rewiring
        if self.final_node == None: # Only rewire if a final path exists
            return False
        candidate_shortcut_positions = np.empty((0,3))
        best_candidate_pos = None
        shortest_path_thusfar = np.inf
        rewire = False
        connection_parent_node_num = None
        connection_child_node_num = None
        num_discrete = 11
        
        # Debug plotting - Uncomment to show "Path Hunter" animation:
        if self.plot_all:
            dot_id = self.env.plot_point(hunter_node.pos, color=Color.BLUE, pointSize=25)
            created_dots.append(dot_id)
            self.graph.lines_plotted += 1
        
        # Add discretization of final path connections to list
        for node in reversed(self.final_node.connections):
            # Skip start node
            if node.dist_from_start == 0:
                continue
            sc_start = node.pos
            sc_end = node.parent.pos
            candidate_shortcut_positions = np.vstack((candidate_shortcut_positions, np.linspace(sc_start, sc_end, num_discrete)))            

        # Now for each node in the path, check for shortcuts at discrete points along the path
        # Need a separate routine for the final node which calculates the resulting path length differently
        if hunter_node.id == self.final_node.id:
            for idx, candidate_pos in enumerate(candidate_shortcut_positions):
                node_num = -(idx // num_discrete) - 2
                # # Debug plotting - Uncomment to show "Path Hunter" animation:
                # if self.plot_all:
                #     dot_id = self.env.plot_point(candidate_pos, color=Color.GREEN, pointSize=15)
                #     created_dots.append(dot_id)
                #     self.graph.lines_plotted += 1
                if self.check_collision_connection(hunter_node.pos, candidate_pos) == False:
                    resulting_path_length = np.linalg.norm(hunter_node.pos - candidate_pos) + \
                        np.linalg.norm(candidate_pos - self.final_node.connections[node_num].pos) + \
                            self.final_node.connections[node_num].dist_from_start
                    if self.final_node.dist_from_start - resulting_path_length > impr_th  and resulting_path_length < shortest_path_thusfar:
                        shortest_path_thusfar = resulting_path_length
                        connection_parent_node_num = node_num
                        best_candidate_pos = candidate_pos
                        rewire = True
        else:
            for idx, candidate_pos in enumerate(candidate_shortcut_positions):
                node_num = -(idx // num_discrete) - 1
                # # Debug plotting - Uncomment to show "Path Hunter" animation:
                # if self.plot_all:
                #     dot_id = self.env.plot_point(candidate_pos, color=Color.GREEN, pointSize=15)
                #     created_dots.append(dot_id)
                #     self.graph.lines_plotted += 1
                if self.check_collision_connection(hunter_node.pos, candidate_pos) == False:
                    resulting_path_length = hunter_node.dist_from_start + \
                        np.linalg.norm(hunter_node.pos - candidate_pos) + \
                            np.linalg.norm(candidate_pos - self.final_node.connections[node_num].pos) + \
                                (self.final_node.dist_from_start - self.final_node.connections[node_num].dist_from_start)
                    if self.final_node.dist_from_start - resulting_path_length > impr_th  and resulting_path_length < shortest_path_thusfar:
                        shortest_path_thusfar = resulting_path_length
                        connection_child_node_num = node_num
                        best_candidate_pos = candidate_pos
                        rewire = True

        # Debug plotting - Uncomment to show "Path Hunter" animation:
        if self.plot_all and self.env.gui:
            for dot in created_dots:
                self.env.remove_line(dot)

        # If we have found a shortcut within the known best path, we will rewire
        if rewire:
            if hunter_node.id == self.final_node.id:
                connection_parent = self.final_node.connections[connection_parent_node_num]
                # Create new intermediary node
                new_node = self.graph.add_node(best_candidate_pos, connection_parent, self.env)
                # Connect new node to original child
                new_node.children.append(hunter_node)
                # Connect parent to new node
                connection_parent.children.append(new_node)
                # Disconnect original pair from parent
                connection_parent.children.remove(self.final_node.connections[connection_parent_node_num+1])
                # Set original child's parent to the new node
                hunter_node.parent = new_node
            else:
                connection_child = self.final_node.connections[connection_child_node_num]
                # Disconnect original pair from parent
                connection_child.parent.children.remove(connection_child)
                # Create new intermediary node
                new_node = self.graph.add_node(best_candidate_pos, hunter_node, self.env)
                # Connect new node to original child
                new_node.children.append(connection_child)
                # Set original child's parent to the new node
                connection_child.parent = new_node
            
            # Need to now recalculate connections and distance to start for each node in new final_node path
            node_under_evaluation = self.final_node
            new_final_connections = []
            new_final_connections.append(node_under_evaluation)
            while node_under_evaluation.dist_from_start > 0:
                node_under_evaluation = node_under_evaluation.parent
                new_final_connections.append(node_under_evaluation)
            new_final_connections.reverse()
            self.final_node.connections = new_final_connections.copy()
            for idx, node in enumerate(self.final_node.connections):
                if idx == 0:
                    node.dist_from_start = 0
                    node.connections = [node]
                else:
                    node.dist_from_start = node.parent.dist_from_start + np.linalg.norm(node.pos - node.parent.pos)
                    node.connections = node.parent.connections + [node]
            self.fastest_route_to_end = self.final_node.dist_from_start
            if self.plot_all and self.env.gui:
                num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.RED)
                self.graph.lines_plotted += num_lines_formed
            # print(f"Path Hunt returns True. New fastest distance: {self.fastest_route_to_end}")
            
            return True

        return False

class RRT(SamplingPlanner):
    def __init__(self,start:Node, goal:Node, space:Space, map, env, result:dict = {}, map_number:int=0):
        super().__init__(start, goal, space, map, env, result, map_number)
        self.name = 'rrt'

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

        new_node = self.graph.add_node(new_node_pos=new_node_pos, parent=closest_node, env=self.env)
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = self.graph.nodes[-1]
            elif self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
                self.final_node = self.graph.nodes[-1]                
            if self.final_node is not None:
                if self.plot_all and self.env.gui:
                    num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
                    self.graph.lines_plotted += num_lines_formed

class Informed_RRT(RRT):
    def __init__(self, start:Node, goal:Node, space:Space, map, env, result:dict = {}, map_number:int=0):
        super().__init__(start, goal, space, map, env, result, map_number)
        # printC(f"Starting WS: UL: {self.space.hl}, LL: {self.space.ll}")
        self.far_nodes_discarded = 0
        self.name = 'inf_rrt'

    def plan(self):
        new_node_pos = self.sample_node_position()
        node_further = self.check_if_further(new_node_pos)
        if node_further:    
            return
        collision_node = self.check_collision_node(new_node_pos)
        if collision_node:
            return
        closest_node = self.find_closest_node(new_node_pos)
        collision_connection = self.check_collision_connection(closest_node.pos, new_node_pos)
        if collision_connection:
            return
        
        new_node = self.graph.add_node(new_node_pos=new_node_pos, parent=closest_node, env=self.env)
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = self.graph.nodes[-1]
                self.constrict_WS()
            # elif self.graph.nodes[-1].dist_from_start < self.final_node.dist_from_start:
            #     self.final_node = self.graph.nodes[-1]
            #     self.constrict_WS()
            if self.final_node is not None:
                if self.plot_all:
                    num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
                    self.graph.lines_plotted += num_lines_formed  
            prev_fastest_length = self.final_node.dist_from_start              
            for _ in range(MAX_IMPR-4):
                for node in self.final_node.connections:
                    self.execute_path_hunt(node)
                    self.num_impr += 1
                if not self.final_node.dist_from_start < prev_fastest_length: break
                else: prev_fastest_length = self.final_node.dist_from_start
            

    def check_if_further(self, new_node_pos):
        if self.final_node is not None:
            if np.linalg.norm(new_node_pos-self.start.pos) + np.linalg.norm(new_node_pos-self.goal.pos) > self.final_node.dist_from_start:
                self.far_nodes_discarded += 1
                # print(f"Discared {self.far_nodes_discarded} nodes so far")
                return True
            else:
                return False

class Recycle_RRT(RRT):
    def __init__(self,start:Node, goal:Node, space:Space, map, env, result:dict = {}, map_number:int=0):
        super().__init__(start, goal, space, map, env, result, map_number)
        self.name = 'rec_rrt'

    def clear_unused_nodes(self):
        used_idxs = []
        printC(f"Initial Number of Nodes {len(self.graph.nodes)}")
        for node in self.final_node.connections:
            used_idxs.append(self.graph.nodes.index(node))
        for i in reversed(range(len(self.graph.nodes))):
            if i not in used_idxs:
                self.graph.nodes.pop(i)
        printC(f"Reduced Number of Nodes {len(self.graph.nodes)}", color=PrintColor.YELLOW)

    def plan(self):
        new_node_pos = self.sample_node_position()
        collision_node = self.check_collision_node(new_node_pos)
        if collision_node:
            return
        closest_node = self.find_closest_node(new_node_pos)
        collision_connection = self.check_collision_connection(closest_node.pos, new_node_pos)
        if collision_connection:
            return
        
        new_node = self.graph.add_node(new_node_pos=new_node_pos, parent=closest_node, env=self.env)
        self.check_reached_goal()
        if self.last_node_in_goal:
            if self.final_node == None: 
                self.final_node = new_node
            elif new_node.dist_from_start < self.final_node.dist_from_start:
                self.final_node = new_node
            if self.final_node is not None:
                if self.plot_all and self.env.gui:
                    num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
                    self.graph.lines_plotted += num_lines_formed
            self.clear_unused_nodes()

class RRT_Star(SamplingPlanner):
    def __init__(self, start: Node, goal: Node, space: Space, map, env, result: dict = {}, map_number:int=0):
        super().__init__(start, goal, space, map, env, result, map_number)
        self.name = 'rrt_star'
    
    def find_lowest_cost_node(self, sample_point: np.ndarray):
        close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - sample_point))[:max(len(self.graph.nodes), 6)]
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
        
        new_node = self.graph.add_node(new_node_pos=new_node_pos, parent=closest_node, env=self.env)

        self.check_shortcut_for_nodes(new_node)
        if (np.linalg.norm(new_node.pos -self.goal.pos)<DIST_TH) and (new_node.dist_from_start < self.fastest_route_to_end):
            if self.final_node is not None and self.env.gui:
                self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
            self.final_node = new_node
            self.fastest_route_to_end = new_node.dist_from_start
            self.reached_goal = True
        
        self.garbage_collection()

    def garbage_collection(self):
        if len(self.graph.nodes) <= 0: return
        if np.linalg.norm(self.graph.nodes[-1].pos -self.goal.pos)<DIST_TH:
            for node in self.graph.nodes:
                if node.dist_from_start + np.linalg.norm(self.goal.pos - node.pos) > self.fastest_route_to_end:
                    self.graph.remove_node(node, self.env, self.final_node)
                    self.nr_nodes_gc += 1
        elif self.graph.nodes[-1].dist_from_start + np.linalg.norm(self.goal.pos - self.graph.nodes[-1].pos) > self.fastest_route_to_end:
            self.graph.remove_node(self.graph.nodes[-1], self.env, self.final_node)
            self.nr_nodes_gc += 1

    def reroute(self, node_s, new_node):
        collision_connection = self.check_collision_connection(node_s.pos, new_node.pos)
        if collision_connection:
            return

        rerouted_node = self.graph.add_node(new_node_pos=node_s.pos, parent=new_node, env=self.env)
        self.graph.remove_node(node_s, self.env, self.final_node)
 
    def check_shortcut_for_nodes(self, new_node):
        close_nodes = sorted(self.graph.nodes, key=lambda n: np.linalg.norm(n.pos - new_node.pos))[1:max(len(self.graph.nodes), 10)]
        for node in close_nodes:
            shortcut_bool = new_node.dist_from_start + np.linalg.norm(node.pos - new_node.pos) < node.dist_from_start - 0.0000001
            if shortcut_bool:
                self.reroute(node, new_node)

class Informed_RRT_Star(RRT_Star):
    def __init__(self,start: Node, goal: Node, space: Space, map, env, result: dict = {}, map_number:int=0):
        super().__init__(start, goal, space, map, env, result, map_number)
        # printC(f"Starting WS: UL: {self.space.hl}, LL: {self.space.ll}")
        self.far_nodes_discarded = 0
        self.name = 'inf_rrt_star'

    def plan(self):
        if self.num_impr >= MAX_IMPR: return
        new_node_pos = self.sample_node_position()
        node_further = self.check_if_further(new_node_pos)
        if node_further:    
            return
        collision_node = self.check_collision_node(new_node_pos)
        if collision_node:
            return

        closest_node = self.find_lowest_cost_node(new_node_pos)
        if closest_node is None:
            return

        new_node = self.graph.add_node(new_node_pos=new_node_pos, parent=closest_node, env=self.env)

        # Each time a new quicker route to the goal is found via a new node in proximity to the goal, execute the below
        if ((np.linalg.norm(new_node.pos -self.goal.pos)<DIST_TH) and (new_node.dist_from_start < self.fastest_route_to_end)):
            if self.final_node == None:
                self.final_node = new_node
                self.reached_goal = True
                self.fastest_route_to_end = new_node.dist_from_start
                if self.final_node is not None:
                    if self.plot_all and self.env.gui:
                        num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
                        self.graph.lines_plotted += num_lines_formed
                self.constrict_WS()
                self.garbage_collection()
                prev_fastest_length = self.final_node.dist_from_start
                for _ in range(MAX_IMPR-4):
                    for node in self.final_node.connections:
                        self.execute_path_hunt(node)
                        self.num_impr += 1
                    if not self.final_node.dist_from_start < prev_fastest_length: break
                    else: prev_fastest_length = self.final_node.dist_from_start
            else:
                self.final_node = new_node
                self.fastest_route_to_end = new_node.dist_from_start
                self.reached_goal = True
                self.constrict_WS()
                if self.final_node is not None:
                    if self.plot_all and self.env.gui:
                        num_lines_formed = self.env.plot_plan(self.final_node.connections, Nodes=True, color=Color.GREEN)
                        self.graph.lines_plotted += num_lines_formed
                for node in self.final_node.connections:
                    self.execute_path_hunt(node)
                    self.num_impr += 1
        self.garbage_collection()

    def check_if_further(self, new_node_pos):
        # Check if a new node position lies within the "shortest path" zone
        if np.linalg.norm(new_node_pos-self.start.pos) + np.linalg.norm(new_node_pos-self.goal.pos) > self.fastest_route_to_end:
            return True
        else:
            # Debug plotting:
            if self.plot_all:
                self.env.plot_point(new_node_pos, color=Color.BLUE, pointSize=4)
                self.graph.lines_plotted += 1
            return False