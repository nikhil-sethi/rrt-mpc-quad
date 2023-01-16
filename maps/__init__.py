from maps.obstacles import Cuboid, Cube
from utils.color import Color
import numpy as np
import pybullet as p
from utils import printC

class Map:

    def __init__(self, map_number = 1, starting_pos = [0.6,-0.6,0.5], goal_pos = [-0.6,0.6,0.5]):
        self.map_indices = [0,1,2,3,4,5,6,7]
        self.starting_pos = starting_pos
        self.goal_pos = goal_pos
        self.num_maps = len(self.map_indices)
        self.dil = 0.0 # Change to allow the workspace to include space around and over the obstacles
        if map_number not in self.map_indices:
            printC(f"Chosen map number not defined. Map options are {self.map_indices}. Resorting to default map, Map 1.")
            self.map_number = 1
        else:
            self.map_number = map_number
        self.load_obstacles()
        self.define_ws_size()

    def load_obstacles(self):
        # Defining often used landing pads:
        self.landing_pads = [
            Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
            Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)
            ]
        if self.map_number == 0:
            self.obstacles = self.landing_pads
        elif self.map_number == 1:
            # SINGLE WALL MAP
            self.dil=0.5
            self.obstacles = self.landing_pads
            self.obstacles += [
                Cuboid(name = "wall_a", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[1.5, 0.1, 0.6], color = Color.BLUEGLASS)
                ]
        elif self.map_number == 2:
            # ZIGZAG MAP
            self.obstacles = self.landing_pads
            self.obstacles += [
                Cuboid(name = "wall_b", origin = [-0.7, -0.2, 0.6], orientation = [0,0, 3.14/4], sides=[1.3, 0.1, 1.2], color = Color.BLUEGLASS),
                Cuboid(name = "wall_c", origin = [0.7, 0.2, 0.6], orientation = [0,0, 3.14/4], sides=[1.3, 0.1, 1.2], color = Color.BLUEGLASS),
                # Cuboid(name = "wall_d", origin = [0, 0, 1.25], orientation = [0, 0, 0], sides=[3, 3, 0.1], color = Color.GLASS)
                ]
        elif self.map_number == 3:
            # COMPLEX BUILDING MAP
            self.obstacles = self.landing_pads
            self.obstacles += [
                Cuboid(name = "wall_b", origin = [-0.3, 0.3, 0.6], orientation = [0, 0, 0], sides=[2, 0.1, 1.2], color = Color.BLUEGLASS),
                Cuboid(name = "wall_c", origin = [0, -0.3, 0.6], orientation = [0, 0, 0], sides=[1.4, 0.1, 1.2], color = Color.BLUEGLASS),
                Cuboid(name = "wall_d", origin = [0.2, 0.925, 0.6], orientation = [0, 0, 0], sides=[0.1, 1.15, 1.2], color = Color.BLUEGLASS),
                Cuboid(name = "wall_e", origin = [-0.2, -0.925, 0.6], orientation = [0, 0, 0], sides=[0.1, 1.15, 1.2], color = Color.BLUEGLASS),
                Cuboid(name = "wall_f", origin = [0, 0, 0.3], orientation = [0, 0, 0], sides=[0.1, 0.5, 0.6], color = Color.REDGLASS),
                Cuboid(name = "wall_g", origin = [-1.25, -0.125, 0.3], orientation = [0, 0, 0], sides=[0.1, 0.75, 0.6], color = Color.REDGLASS),
                Cuboid(name = "wall_h", origin = [-1.25, -1.39, 0.3], orientation = [0, 0, 0], sides=[0.1, 0.225, 0.6], color = Color.REDGLASS),
                Cuboid(name = "wall_i", origin = [-1.25, -0.625, 0.9], orientation = [0, 0, 0], sides=[0.1, 1.75, 0.6], color = Color.REDGLASS),
                Cuboid(name = "wall_j", origin = [-1.65, 0.3, 0.4], orientation = [0, 0, 0], sides=[0.7, 0.1, 0.8], color = Color.REDGLASS),
                # Outer walls & ceiling:
                Cuboid(name = "wall_top", origin = [-0.25, 0, 1.25], orientation = [0, 0, 0], sides=[3.7, 3.2, 0.1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_north", origin = [-0.25, 1.55, 0.6], orientation = [0, 0, 0], sides=[3.7, 0.1, 1.2], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_west", origin = [-2.05, 0, 0.6], orientation = [0, 0, 0], sides=[0.1, 3, 1.2], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_south", origin = [-0.25, -1.55, 0.6], orientation = [0, 0, 0], sides=[3.7, 0.1, 1.2], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_east", origin = [1.55, 0, 0.6], orientation = [0, 0, 0], sides=[0.1, 3, 1.2], color = Color.LIGHT_GLASS)
                ]
        elif self.map_number == 4:
            # SCATTERED OBJECTS
            self.starting_pos = [2,-2,0.5]
            self.goal_pos = [-2,2,0.5]
            self.obstacles = [
                Cube(name = "landing2", origin=[-2, 2, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off2", origin=[2, -2, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]
            self.obstacles += [
                Cuboid(name = "pillar_01", origin = [0, 0, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_02", origin = [-1.5, -1.2, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_03", origin = [-1.5, 0.4, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_04", origin = [-1.2, 1.2, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_05", origin = [-1.2, -1.3, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_06", origin = [-0.9, 1.8, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_07", origin = [-0.9, 1.4, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_08", origin = [-0.6, 1.5, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_09", origin = [-0.6, -0.3, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_7", origin = [-0.3, -0.9, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_11", origin = [0, -0.9, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_12", origin = [0, 0.9, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_13", origin = [0.3, -0.3, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_14", origin = [0.3, -1.5, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_15", origin = [0.6, 0.5, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_16", origin = [0.6, 1.2, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_17", origin = [0.9, -0.9, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_18", origin = [0.9, -1.7, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_19", origin = [1.2, 0.7, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_20", origin = [1.2, -0.7, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_21", origin = [1.5, 1.7, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
                Cuboid(name = "pillar_22", origin = [1.5, -0.1, 0.75], orientation = [0, 0, 0], sides=[0.1, 0.1, 1.5], color = Color.REDGLASS),
            ]
        elif self.map_number == 5:
            # EXTRA: TUNNEL
            self.dil = 0.5
            self.starting_pos = [0.6,-1,0.25]
            self.goal_pos = [0.6,1,1.35]
            self.obstacles = [
                Cube(name = "tun_landing", origin=[0.6,-1, 0.05], orientation = [0,0,0], sides=[0.3, 0.3, 0.1], color = Color.GREEN),
                Cube(name = "tun_take_off", origin=[0.6,1, 1.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.1], color = Color.RED)
            ]
            self.obstacles += [
                Cuboid(name = "wall_m", origin = [0, -1, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_n", origin = [1, -1, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_o", origin = [0.5, -1, 1.05], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_p", origin = [0, 0, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_e", origin = [1, 0, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_f", origin = [0.5, 0, 1.05], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_g", origin = [0, 1, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_h", origin = [1, 1, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_i", origin = [0.5, 1, 1.05], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_j", origin = [0, 2, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_k", origin = [1, 2, 0.5], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_l", origin = [0.5, 2, 1.05], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_mm", origin = [0, -1, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_nm", origin = [1, -1, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_om", origin = [0.5, -1, 2.15], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_pm", origin = [0, 0, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_em", origin = [1, 0, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_fm", origin = [0.5, 0, 2.15], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_gm", origin = [0, 1, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_hm", origin = [1, 1, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_im", origin = [0.5, 1, 2.15], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_jm", origin = [0, 2, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.LIGHT_GLASS),
                Cuboid(name = "wall_km", origin = [1, 2, 1.6], orientation = [0, 0, 3.14/2], sides=[1, 0.1, 1], color = Color.BLACK_GLASS),
                Cuboid(name = "wall_lm", origin = [0.5, 2, 2.15], orientation = [3.14/2, 0, 0], sides=[1.1, 0.1, 1], color = Color.BLACK_GLASS)
                ]
        elif self.map_number == 6:
            # RANDOM SCATTERED
            size = 2
            self.starting_pos = [size,-size,0.5]
            self.goal_pos = [-size,size,0.5]
            origins = []
            sizes = []
            self.obstacles = [
                Cube(name = "landing2", origin=[-size, size, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off2", origin=[size, -size, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]
            origins.append([-size, size, 0.6])
            origins.append([size, -size, 0.6])
            sizes.append([0.3, 0.3, 0.3])
            sizes.append([0.3, 0.3, 0.3])
            for i in range(40):
                x_pos = np.random.uniform(low=-size*1.1, high=size*1.1)
                y_pos = np.random.uniform(low=-size*1.1, high=size*1.1)
                box_sides = 0.15
                sides = [box_sides, box_sides, 1.5]
                x_min = x_pos - sides[0]/2
                x_max = x_pos + sides[0]/2
                y_min = y_pos - sides[1]/2
                y_max = y_pos + sides[1]/2
                overlap = False
                for j in range(len(origins)):
                    obs_x_min = origins[j][0] - sizes[j][0]/2
                    obs_x_max = origins[j][0] + sizes[j][0]/2
                    obs_y_min = origins[j][1] - sizes[j][1]/2
                    obs_y_max = origins[j][1] + sizes[j][1]/2
                    if \
                        (obs_x_min < x_min < obs_x_max or obs_x_min < x_max < obs_x_max) \
                            and \
                                (obs_y_min < y_min < obs_y_max or obs_y_min < y_max < obs_y_max):
                                overlap = True
                                break
                if overlap == False:
                    origins.append([x_pos, y_pos, 0.75])
                    sizes.append(sides.copy())
                    self.obstacles += [Cuboid(name = f"random_pillar_{i}", origin = [x_pos, y_pos, 0.75], orientation = [0, 0, 0], sides=sides.copy(), color = Color.REDGLASS)]
        elif self.map_number == 7:
            # WINDOW
            self.dil=0.5
            self.starting_pos = [0,0,0.5]
            self.goal_pos = [0,3,0.5]
            origins = []
            sizes = []
            self.obstacles = [
                Cube(name = "landing2", origin=[0, 3, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
                Cube(name = "take_off2", origin=[0, 0, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED),
                Cuboid(name = "w0", origin=[0, 1, 0.15], orientation = [0,0,0], sides=[5, 0.3, 0.3], color = Color.BLUEGLASS),
                Cuboid(name = "w1", origin=[-1.35, 1, 0.4], orientation = [0,0,0], sides=[2.3, 0.3, 0.2], color = Color.BLUEGLASS),
                Cuboid(name = "w2", origin=[1.35, 1, 0.4], orientation = [0,0,0], sides=[2.3, 0.3, 0.2], color = Color.BLUEGLASS),
                Cuboid(name = "w3", origin=[0, 1, 2], orientation = [0,0,0], sides=[5, 0.3, 3], color = Color.BLUEGLASS),
                Cuboid(name = "w4", origin=[0, 2, 0.45], orientation = [0,0,0], sides=[2, 0.3, 0.9], color = Color.BLUEGLASS)
                ]


    def define_ws_size(self):
        ws_dilation = np.array([self.dil, self.dil, self.dil])
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

    def load_map(self, client, dilate=False, dilation=0.0):
        for obs in self.obstacles:
            obs.load_urdf(client)
            if dilate:
                obs.dilate_obstacles(dilation)
        if dilate:
            self.define_ws_size()

    def view_map(self):
        """Use this function in isolation just to test!! Not with the simulation"""
        client = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(cameraDistance=3,
                                         cameraYaw=-30,
                                         cameraPitch=-30,
                                         cameraTargetPosition=[0, 0, 0],
                                         physicsClientId=client
                                         )
        self.load_map(client=client)

if __name__=="__main__":
    map = Map(map_number=1)
    map.view_map()