import xml.etree.ElementTree as ET
import  enum
import numpy as np

class Color(enum.Enum):
    # RGBA tuple
    RED = (1,0,0,1)
    GREEN = (0,1,0,1)
    BLUE = (0,0,1,1)
    GRAY = (0.8,0.8,0.8,1)
    GLASS = (0.8,0.8,0.8,0.7)
    
def node2dict(node):
    d = node.attrib
    for child in node:
        key =  child.tag
        val = node2dict(child)
        d.update({key:val})
    return d

def discretize_path(connections, num_steps=200) -> list:
    # each node in connections has a position (list of x,y,z)
    # need to define a line/vector from parent pos to node position
    traj = np.empty((0,3))
    for i in range(len((connections))-1):
        node_start = np.asarray(connections[i].pos)
        node_end = np.asarray(connections[i+1].pos)
        traj_btw_nodes = np.linspace(node_start, node_end, num=num_steps, endpoint=False)
        traj = np.append(traj, traj_btw_nodes, axis=0)
    traj = np.append(traj, np.asarray(connections[-1].pos).reshape(1,3), axis=0)
    return traj

# def urdf2obs(file):
#     urdf_obs = ET.parse(file).getroot()
#     # d = node2dict(urdf_obs)
#     col = urdf_obs.find("link").find("collision")
#     obs = Obstacle3D()
#     origin = [float(x) for x in col.find("origin").attrib["xyz"].split(" ")]
#     obs.origin = tuple(origin)

#     bbox = [float(x) for x in col.find("geometry").find("box").attrib["size"].split(" ")]
#     obs.bbox = tuple(bbox)
    
#     return obs

class Quadrotor:
    def __init__(self):
        self.radius = 0.08  # [m]
        self.pos = np.array([0., 0., 0.])
