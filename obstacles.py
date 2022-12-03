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

    @property
    def extent(self):
        return (self.pose.origin - self.bbox, self.pose.origin + self.bbox)

    # def is_colliding(self, point:list):
    #     extent = self.extent
    #     if extent[0] < point[0] <

class Cuboid(Obstacle3D):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        super().__init__(name, origin, orientation, sides, color=color)

class Cube(Obstacle3D):
    def __init__(self, name, origin, orientation, sides:tuple, color = Color.GRAY) -> None:
        assert sides[0] == sides[1] == sides[1]
        super().__init__(name, origin, orientation, sides, color=color)
