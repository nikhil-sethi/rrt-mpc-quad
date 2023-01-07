from maps.obstacles import Cuboid, Cube
from utils.color import Color
import numpy as np

class Map:

    def __init__(self, map_number=0):
        if map_number not in [0,1,2,3]:
            print("Chosen map number not defined. Resorting to default map, Map 0.")
            self.map_number = 0
        else:
            self.map_number = map_number
        self.load_obstacles()

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
        all_ll = []
        all_ul = []
        for obs in self.obstacles:
            all_ll.append(obs.extent[0].flatten())
            all_ul.append(obs.extent[1].flatten())
        all_ll = np.array(all_ll)
        all_ul = np.array(all_ul)
        print(all_ll)
        print(all_ul)
        self.ws_ll = np.min(all_ll, axis = 0)
        self.ws_ul = np.max(all_ul, axis = 0)

    def load_map(self, client, dilate=False, dilation=0):
        for obs in self.obstacles:
            obs.load_urdf(client)
            if dilate:
                obs.dilate_obstacles(dilation)