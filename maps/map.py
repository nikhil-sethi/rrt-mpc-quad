from maps.obstacles import Cuboid, Cube
from utils.color import Color
import numpy as np

class Map:

    def __init__(self, map_number = 1):
        self.map_indices = [0,1,2,3]
        self.num_maps = len(self.map_indices)
        if map_number not in self.map_indices:
            print(f"Chosen map number not defined. Map options are {self.map_indices}. Resorting to default map, Map 1.")
            self.map_number = 1
        else:
            self.map_number = map_number
        self.load_obstacles()
        self.define_ws_size()

    def load_obstacles(self):
        if self.map_number == 0:
            self.starting_pos = [0.6,-0.6,0.5]
            self.goal_pos = [-0.6,0.6,0.5]
            self.obstacles = [
                Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]
        elif self.map_number == 1:
            self.starting_pos = [0.6,-0.6,0.5]
            self.goal_pos = [-0.6,0.6,0.5]
            self.obstacles = [
                Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED),
                Cuboid(name = "wall_a", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[0.7, 0.1, 0.6], color = Color.GLASS)]
        elif self.map_number == 2:
            self.starting_pos = [0.6,-0.6,0.5]
            self.goal_pos = [-0.6,0.6,0.5]
            self.obstacles = [
                Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED),
                Cuboid(name = "wall_b", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[2, 0.1, 0.6], color = Color.BLUE)]
        elif self.map_number == 3:
            self.starting_pos = [0.6,-0.6,0.5]
            self.goal_pos = [-0.6,0.6,0.5]
            self.obstacles = [
                Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED),
                Cuboid(name = "wall_c", origin = [0, 0, 0.35], orientation = [0,0, 3.14/4], sides=[2, 0.1, 0.7], color = Color.BLUE),
                Cuboid(name = "wall_d", origin = [0, 0, 1.2], orientation = [0,0, 3.14/4], sides=[1, 0.1, 1], color = Color.BLUE)]

    def define_ws_size(self):
        ws_dilation = np.array([0.5, 0.5, 0.5])
        all_limits = []
        for obs in self.obstacles:
            all_limits.append(obs.extent[0].flatten())
            all_limits.append(obs.extent[1].flatten())
            # print(f"{obs.name} LL = {obs.extent[0].flatten()}")
            # print(f"{obs.name} UL = {obs.extent[1].flatten()}")
        # Also include starting and goal positions, in case they do note correspond to obstacles
        all_limits.append(self.starting_pos)
        all_limits.append(self.goal_pos)
        all_limits = np.array(all_limits)
        self.ws_ll = np.min(all_limits, axis = 0) - ws_dilation
        self.ws_ul = np.max(all_limits, axis = 0) + ws_dilation
        # Exclude ws below floor
        if self.ws_ll[2] <= 0: self.ws_ll[2] = 0
        # print(f"all_limits = \n{all_limits}")
        # print(f"ws_ll = {self.ws_ll}")
        # print(f"ws_ul = {self.ws_ul}")

    def load_map(self, client, dilate=False, dilation=0):
        for obs in self.obstacles:
            obs.load_urdf(client)
            if dilate:
                obs.dilate_obstacles(dilation)
        if dilate:
            self.define_ws_size()