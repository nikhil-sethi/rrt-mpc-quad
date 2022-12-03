from obstacles import Cuboid, Cube
from utils import Color

MAP_0 = [Cube(name = "landing", origin=[-0.6, 0.6, 0.3], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.GREEN),
        Cube(name = "take_off", origin=[0.6, -0.6, 0.3], orientation = [0,0,0], sides=[0.3, 0.3, 0.3], color = Color.RED)]

MAP_1 = MAP_0.copy()
MAP_1.extend([
    Cuboid(name = "wall_a", origin = [0, 0, 0.5], orientation = [0,0, 3.14/4], sides=[0.7, 0.1, 0.6], color = Color.GLASS)
])

MAPS = [MAP_0, MAP_1]


def load_map(map:list, client):
    for obs in map:
        obs.load_urdf(client)
