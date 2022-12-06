from obstacles import Cuboid, Cube
from utils import Color

MAP_0 = [Cube(name = "landing", origin=[-0.6, 0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
        Cube(name = "take_off", origin=[0.6, -0.6, 0.15], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]

MAP_1 = MAP_0 + [
    Cuboid(name = "wall_a", origin = [0, 0, 0.3], orientation = [0,0, 3.14/4], sides=[0.7, 0.1, 0.6], color = Color.GLASS)
]

MAP_2 = MAP_0 + [
    Cuboid(name = "wall_b", origin = [0, 0, 1], orientation = [0,0, 3.14/4], sides=[3, 0.1, 2], color = Color.BLUE)
]

MAPS = [MAP_0, MAP_1, MAP_2]


def load_map(map:list, client):
    for obs in map:
        obs.load_urdf(client)
        
# add msp display