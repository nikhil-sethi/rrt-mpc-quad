import pybullet as p
import os

def load_takeoff_land(client):
    # Takeoff base
    p.loadURDF(os.getcwd() + "/maps/take_off_base.urdf",
                [0.6, -0.6, 0.3],
                p.getQuaternionFromEuler([0, 0, 0]),
                physicsClientId=client
                )

    # End Landing base
    p.loadURDF(os.getcwd() + "/maps/landing_base.urdf",
                [-0.6, 0.6, 0.3],
                p.getQuaternionFromEuler([0, 0, 0]),
                physicsClientId=client
                )


def load_map(i, client):
    load_takeoff_land(client)
    if i == 1: # Map 1
        # wall A
        p.loadURDF(os.getcwd() + "/maps/wall_a.urdf",
                [0, 0, 0.5],
                p.getQuaternionFromEuler([0, 0, 0.785]),
                physicsClientId=client
                )
