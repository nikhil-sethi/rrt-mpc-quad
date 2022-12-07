import sys
sys.path.insert(0, "/home/matthijs/Softwares/forces_pro_client")
import numpy as np

# import casadi
import forcespro
import forcespro.nlp
import matplotlib.pyplot as plt
import matplotlib.patches
from matplotlib.gridspec import GridSpec
import casadi


def continuous_dynamics_3d(x, u):
    m = 0.027 # [kg]
    g = 9.81 # [m/s^2]
    Ixx = 1.4e-5 # [kg/m^2]
    Iyy = 1.4e-5 # [kg/m^2]
    Izz = 2.17e-5 # [kg/m^2]

    L = 0.0397 # [m]
    k_F = 3.16e-10 # [N/rpm^2]
    k_M = 7.94e-12 # [N/rpm^2]
    # rpm to force:
    F1 = k_F * u[0]**2
    F2 = k_F * u[1]**2
    F3 = k_F * u[2]**2
    F4 = k_F * u[3]**2
    #forces to thrust and moments:
    u1 = F1 + F2 + F3 + F4
    u2 = F2 * L - F4 * L
    u3 = -F1 * L + F3 * L
    u4 = k_M/k_F * F1 - k_M/k_F * F2 + k_M/k_F * F3 - k_M/k_F * F4
    x_dot_0 = x[3]
    x_dot_1 = x[4]
    x_dot_2 = x[5]
    x_dot_3 = ((casadi.cos(x[8])*casadi.sin(x[7]) + casadi.cos(x[7])*casadi.sin(x[6])*casadi.sin(x[8])) / m) * u1
    x_dot_4 = ((casadi.sin(x[8])*casadi.sin(x[7]) - casadi.cos(x[7])*casadi.sin(x[6])*casadi.cos(x[8])) / m) * u1
    x_dot_5 = ((casadi.cos(x[7])*casadi.cos(x[6])) / m) * u1 - g
    x_dot_6 = x[9]
    x_dot_7 = x[10]
    x_dot_8 = x[11]
    x_dot_9 =  -((-Iyy + Izz) / Ixx) * x[7] * x[8] + 1/Ixx * u2
    x_dot_10 = -((Ixx - Izz) / Iyy) * x[6] * x[8] + 1/Iyy * u3
    x_dot_11 = -((-Ixx + Iyy) / Izz) * x[6] * x[7] + 1/Izz * u4


    return casadi.vertcat(x_dot_0, x_dot_1, x_dot_2, x_dot_3, 
                          x_dot_4, x_dot_5, x_dot_6, x_dot_7,
                          x_dot_8, x_dot_9, x_dot_10, x_dot_11)                           # ddelta/dt = phi


def generate_pathplanner():
    """Generates and returns a FORCESPRO solver that calculates a path based on
    constraints and dynamics while minimizing an objective function
    """
    # Model Definition
    # ----------------

    # Problem dimensions
    model = forcespro.nlp.SymbolicModel()
    model.N = 10  # horizon length
    model.nvar = 16  # number of variables
    model.neq = 12  # number of equality constraints
    model.npar = 3 # number of runtime parameters

    model.objective = obj
    model.objectiveN = objN 
    integrator_stepsize = 0.1
    model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics_3d, z[4:16], z[0:4],
                                                integrator=forcespro.nlp.integrators.RK4,
                                                stepsize=integrator_stepsize)

    model.E = np.concatenate([np.zeros((12, 4)), np.eye(12)], axis=1)


    #                     inputs                 |  states
    #                 thrust       Mx       My       Mz        x        y        z       vx       vy       vz     roll    pitch      yaw    roll_rate   pitch_rate    yaw_rate
    model.lb = np.array([0.0,   0.0,     0.0,    0.0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,   -0.5, -0.5, -np.pi,    -5*np.pi, -5*np.pi, -5*np.pi])
    model.ub = np.array([30000,    30000, 30000, 30000,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,    0.5,0.5,  np.pi,   5*np.pi, 5*np.pi, 5*np.pi])

    # Initial condition on vehicle states x
    model.xinitidx = range(4,16) # use this to specify on which variables initial conditions

    codeoptions = forcespro.CodeOptions('FORCESNLPsolver')
    codeoptions.maxit = 1000     # Maximum number of iterations
    codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but 
    #                             not for timings)
    codeoptions.optlevel = 3    # 0 no optimization, 1 optimize for size, 
    #                             2 optimize for speed, 3 optimize for size & speed
    codeoptions.nlp.TolStat = 0.1 
    codeoptions.forcenonconvex = 1
    codeoptions.noVariableElimination = 1
    codeoptions.nohash = 1
    codeoptions.cleanup = False
    codeoptions.timing = 1
    # codeoptions.nlp.hessian_approximation = 'bfgs'
    # codeoptions.solvemethod = 'SQP_NLP' # choose the solver method Sequential 
    # #                              Quadratic Programming
    # codeoptions.nlp.bfgs_init = 2.5*np.identity(16)
    # codeoptions.sqp_nlp.maxqps = 1      # maximum number of quadratic problems to be solved
    # codeoptions.sqp_nlp.reg_hessian = 5e-9 # increase this if exitflag=-8
    # change this to your server or leave uncommented for using the 
    # standard embotech server at https://forces.embotech.com 
    # codeoptions.server = 'https://forces.embotech.com'
    
    # Creates code for symbolic model formulation given above, then contacts 
    # server to generate new solver
    solver = model.generate_solver(options=codeoptions)

    return model,solver

def obj(z,current_target):
    #return 1000.0 * (z[6])**2
    # return (1000.0 * (z[6])**2 + 500 * (5 - z[7])**2  +  0.1*z[0]**2 + 0.1*z[1]**2 + 0.1*z[2]**2 + 0.1*z[3]**2)
    return (100.0*(z[4]-current_target[0])**2 + 100.0*(z[5]-current_target[1])**2 + 100.0*(z[6]-current_target[2])**2 +
            10*(z[0]/20000)**2 + 10*(z[1]/20000)**2 + 0.1*(z[2]/20000)**2 + 10*(z[3]/20000)**2)

def objN(z,current_target):
    #return 2000 * (z[6])**2
    # return 2*(1000.0 * (z[6])**2 + 500 * (5 - z[7])**2 +  0.1*z[0]**2 + 0.1*z[1]**2 + 0.1*z[2]**2 + 0.1*z[3]**2)
    return (200.0*(z[4]-current_target[0])**2 + 200.0*(z[5]-current_target[1])**2 + 200.0*(z[6]-current_target[2])**2 +
            20*(z[0]/20000)**2 + 20*(z[1]/20000)**2 + 20*(z[2]/20000)**2 + 20*(z[3]/20000)**2)

def find_closest_point(points, ref_point):
    return  np.argmin(np.linalg.norm(points - ref_point, axis=1))

def extract_next_path_points(path_points, pos, N):
    closest_point = find_closest_point(path_points, pos)
    n_points = path_points.shape[0]
    next_path_points = list(path_points[closest_point+1:min(closest_point+N+1, n_points)]) + \
        [path_points[-1]] * int(max(0, closest_point + N + 1 - n_points))
    return next_path_points


def main():
    model, solver = generate_pathplanner()

    sim_length = 100 # simulate 8sec

    x = np.zeros((12,sim_length+1)) # states
    u = np.zeros((4,sim_length)) # inputs

    # Set initial guess to start solver from
    x0i = np.zeros((model.nvar,1))
    x0 = np.transpose(np.tile(x0i, (1, model.N)))
    # Set initial condition
    xinit = np.transpose(np.array([5, 0., 0., 0., 0., 0., 0., 0., np.deg2rad(90), 0., 0., 0.]))
    x[:,0] = xinit

    problem = {"x0": x0,
            "xinit": xinit}

    num_points = 200
    path_points = np.array([[5*np.cos(theta), 10*np.sin(theta), 0.0] for theta in np.linspace(0,2*np.pi, num_points)])
    #path_points = np.array([[5 + theta, 0.0, 0.0] for theta in np.linspace(0,70, num_points)])
    start_pred = np.reshape(problem["x0"],(16,model.N)) # first predicition corresponds to initial guess

   
    # Simulation
    for k in range(sim_length):
        
        # Set initial condition
        problem["xinit"] = x[:,k]

        # Set runtime parameters (here, the next N points on the path)
        next_path_points = extract_next_path_points(path_points, x[0:3,k], model.N)
        problem["all_parameters"] = np.reshape(next_path_points, \
            (model.npar*model.N,1))

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)

        # Make sure the solver has exited properly.
        # assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(model.nvar), model.N))
        for i in range(0, model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        problem["x0"] = temp
        pred_u = temp[0:4, :]
        pred_x = temp[4:16, :]

        # Apply optimized input u of first stage to system and save simulation data
        u[:,k] = pred_u[:,0]
        print(u[:,k])
        x[:,k+1] = np.transpose(model.eq(np.concatenate((u[:,k],x[:,k]))))
        print(x[6,k+1], x[7,k+1], x[8,k+1])
    print("mean", np.mean(u[0,:]))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(x[0, :], x[1, :], x[2, :], "g") 
    ax.plot3D(path_points[:, 0], path_points[:, 1], path_points[:, 2], "r")   

    ax.set_zlim(-2, 2)
    plt.show()




if __name__ == "__main__":
    main()
