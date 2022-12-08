from solver import MPCSolver
import numpy as np
import matplotlib.pyplot as plt


class Drone:
    def __init__(self):
        self.max_rotor_speed = 30000 
        self.max_pitch_roll = 0.5
        self.max_pitch_roll_rate = 5 * np.pi
        self.max_yaw_rate = 5 * np.pi

def main():
    drone = Drone()
    mpc_solver = MPCSolver(drone, dt=0.1)

    sim_length = 180 # simulate 8sec

    x = np.zeros((12,sim_length+1)) # states
    u = np.zeros((4,sim_length)) # inputs

    state_init = np.transpose(np.array([5, 0., 0., 0., 0., 0., 0., 0., np.deg2rad(90), 0., 0., 0.]))
    x[:,0] = state_init


    num_points = 200
    path_points = np.array([[5*np.cos(theta), 10*np.sin(theta), 0.0] for theta in np.linspace(0,2*np.pi, num_points)])
    #path_points = np.array([[5 + theta, 0.0, 0.0] for theta in np.linspace(0,70, num_points)])

    for k in range(sim_length):
        u[:,k], x[:,k+1]= mpc_solver.compute_action(x[:, k], path_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(x[0, :], x[1, :], x[2, :], "g") 
    ax.plot3D(path_points[:, 0], path_points[:, 1], path_points[:, 2], "r")   

    ax.set_zlim(-2, 2)
    plt.show()

if __name__=="__main__":
    main()
