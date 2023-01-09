import pybullet as p
import os
from maps.vector import Pose
from utils.color import Color
import numpy as np
from scipy.spatial.transform import Rotation as R

class Obstacle3D:
    """An intermediary class which helps management between python and pybullet obstacles"""

    def __init__(self, name, origin, orientation, bbox, mesh = "cube.obj", color = Color.GRAY) -> None:
        self.name = name
        self.pose = Pose(origin, orientation)
        self.bbox = bbox
        assert bbox[0] > 0, "Sides of the bounding box should be positive"
        assert bbox[1] > 0, "Sides of the bounding box should be positive"
        assert bbox[2] > 0, "Sides of the bounding box should be positive"
        self.mesh = mesh
        self.color:tuple = color # RGBA
        self.T = np.array([])
        self.T_inv = np.array([])
        self.transformed_point = np.ones((4,1))

        self.extent = self.get_extent()
        self.create_urdf()

    def create_urdf(self):
        # create a urdf which pybullet can work with
        archetype = \
        f"""<?xml version="0.0" ?>
        <robot name="{self.name}.urdf"> 
        <link name="baseLink">
            <contact>
                <lateral_friction value="1.0"/>
                <inertia_scaling value="3.0"/>
            </contact>
            <inertial>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <mass value=".1"/>
                <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
            </inertial>
            <visual>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <geometry>
                    <mesh filename="{self.mesh}" scale="{self.bbox[0]} {self.bbox[1]} {self.bbox[2]}"/>
                </geometry>
                <material name="white">
                    <color rgba="{self.color.value[0]} {self.color.value[1]} {self.color.value[2]} {self.color.value[3]}"/>
                </material>
            </visual>
            <collision>
                <origin rpy="0 0 0" xyz="0 0 0"/>
                <geometry>
                    <box size="{self.bbox[0]} {self.bbox[1]} {self.bbox[2]}"/>
                </geometry>
            </collision>
        </link>
        </robot>
        """
        with open(f"./maps/{self.name}.urdf", 'w') as f:
            f.write(archetype)
        

    def load_urdf(self, client):
        p.loadURDF(os.getcwd() + f"/maps/{self.name}.urdf",
            self.pose.origin,
            p.getQuaternionFromEuler(self.pose.orient),
            physicsClientId=client
            )

    def get_extent(self):
        r = R.from_euler('xyz', self.pose.orient, degrees=False).as_matrix()
        T_A_B = np.eye(4)
        T_A_B[0:3,0:3] = r
        T_A_B[0:3,3] = np.array(self.pose.origin)
        self.T = T_A_B
        self.T_inv = np.linalg.inv(self.T)

        pos_half_bbox = np.array([self.bbox[0]/2, self.bbox[1]/2, self.bbox[2]/2, 1])
        neg_half_bbox = np.array([-self.bbox[0]/2, -self.bbox[1]/2, -self.bbox[2]/2, 1])
        ll = (T_A_B@neg_half_bbox.reshape((4,1)))[0:3]
        ul = (T_A_B@pos_half_bbox.reshape((4,1)))[0:3]
        # Rotations greater than 90 degrees cause +- bounding box calculations to flip. So, I manually flip them back, like a dum-dum
        for i in range(ll.shape[0]):
            if ll[i] > ul[i]:
                temp = ul[i].copy()
                ul[i] = ll[i].copy()
                ll[i] = temp
        return (ll, ul)

    def dilate_obstacles(self, dilation: float):
        for i in range(len(self.extent[0])):
            # Reduce the lower-limits by dilation value (drone radius + margin)
            self.extent[0][i] = self.extent[0][i] - dilation
            # Increase the upper-limits by dilation value (drone radius + margi>
            self.extent[1][i] = self.extent[1][i] + dilation
            self.bbox[i] = self.bbox[i] + 2*dilation

class Cuboid(Obstacle3D):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        super().__init__(name, origin, orientation, sides, color=color)
    
    def is_colliding(self, point:np.ndarray):
        self.transformed_point[0:3] = point[:,None]
        self.transformed_point[3] = 1
        self.transformed_point = self.T_inv@self.transformed_point
        for i in range(point.shape[0]):
            if ~(-self.bbox[i]/2 < self.transformed_point[i] < self.bbox[i]/2):
                return False
        return True
        #ret =  all([-self.bbox[i]/2 < self.transformed_point[i] < self.bbox[i]/2 for i in range(point.shape[0])])
        # print(transformed_point, self.name, self.extent, ret)
        #return ret
        
class Cube(Cuboid):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        assert sides[0] == sides[1] == sides[1]
        super().__init__(name, origin, orientation, sides, color=color)
