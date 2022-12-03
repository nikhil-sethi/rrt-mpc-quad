import xml.etree.ElementTree as ET
import  enum

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
