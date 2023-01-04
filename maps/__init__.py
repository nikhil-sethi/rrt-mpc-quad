from obstacles import Cuboid, Cube
from utils import Color

MAP_0 = [Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
        Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]

MAP_1 = MAP_0 + [
    Cuboid(name = "wall_a", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[0.7, 0.1, 0.6], color = Color.GLASS)
]

MAP_2 = MAP_0 + [
    Cuboid(name = "wall_b", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[2, 0.1, 0.6], color = Color.BLUE),
]

MAP_3 = MAP_0 + [
    Cuboid(name = "wall_c", origin = [0, 0, 0.35], orientation = [0,0, 3.14/4], sides=[2, 0.1, 0.7], color = Color.BLUE),
    Cuboid(name = "wall_d", origin = [0, 0, 1.2], orientation = [0,0, 3.14/4], sides=[1, 0.1, 1], color = Color.BLUE)
]

MAPS = [MAP_0, MAP_1, MAP_2, MAP_3]


def load_map(map:list, client, dilate=False, dilation=0):
    for obs in map:
        obs.load_urdf(client)
        if dilate:
            obs.dilate_obstacles(dilation)
