import pybullet as p
import os
from vector import Pose
from utils import Color

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
        ll = [self.pose.origin[i] - self.bbox[i]/2 for i in range(3)]
        ul = [self.pose.origin[i] + self.bbox[i]/2 for i in range(3)]
        return (ll, ul)

    def is_colliding(self, point:list):
        extent = self.extent
        x_in = extent[0][0] < point[0] < extent[1][0]
        y_in = extent[0][1] < point[1] < extent[1][1]
        z_in = extent[0][2] < point[2] < extent[1][2]
        if x_in and y_in and z_in:
            return True
        return False

    def dilate_obstacles(self, dilation: float):
        for i in range(len(self.extent[0])):
            # Reduce the lower-limits by dilation value (drone radius + margin)
            self.extent[0][i] = self.extent[0][i] - dilation
            # Increase the upper-limits by dilation value (drone radius + margi>
            self.extent[1][i] = self.extent[1][i] + dilation

        
class Cuboid(Obstacle3D):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        super().__init__(name, origin, orientation, sides, color=color)

class Cube(Obstacle3D):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        assert sides[0] == sides[1] == sides[1]
        super().__init__(name, origin, orientation, sides, color=color)
