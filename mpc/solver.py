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

from dynamics import continuous_nonlinear_dynamics_3d as continuous_dynamics
from util import extract_next_path_points, find_closest_point
from objectives import obj, objN

class MPCSolver:
    def __init__(self, drone, dt):
        self.dt = dt
        self.model = forcespro.nlp.SymbolicModel()
        self.max_rotor_speed = drone.max_rotor_speed  
        self.max_pitch_roll = drone.max_pitch_roll 
        self.max_pitch_roll_rate = drone.max_pitch_roll_rate
        self.max_yaw_rate = drone.max_yaw_rate
        self.x0 = np.array([])

    def generate_pathplanner(self):        
        self.model.N = 10  # horizon length
        self.model.nvar = 16  # number of variables
        self.model.neq = 12  # number of equality constraints
        self.model.npar = 3 # number of runtime parameters

        self.model.objective = obj
        self.model.objectiveN = objN 
        integrator_stepsize = 0.1
        self.model.eq = lambda z: forcespro.nlp.integrate(continuous_dynamics, z[4:16], z[0:4],
                                                    integrator=forcespro.nlp.integrators.RK4,
                                                    stepsize=integrator_stepsize)

        self.model.E = np.concatenate([np.zeros((12, 4)), np.eye(12)], axis=1)


        #                     inputs                 |  states
        #                 thrust       Mx       My       Mz        x        y        z       vx       vy       vz     roll    pitch      yaw    roll_rate   pitch_rate    yaw_rate
        self.model.lb = np.array([0.0,                                    0.0,                  0.0,                  0.0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,   -self.max_pitch_roll, -self.max_pitch_roll, -np.pi,    -5*np.pi, -5*np.pi, -5*np.pi])
        self.model.ub = np.array([self.max_rotor_speed,  self.max_rotor_speed, self.max_rotor_speed, self.max_rotor_speed,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,    self.max_pitch_roll, self.max_pitch_roll,  np.pi,   5*np.pi, 5*np.pi, 5*np.pi])

        # Initial condition on vehicle states x
        self.model.xinitidx = range(4,16) # use this to specify on which variables initial conditions

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
        self.solver = self.model.generate_solver(options=codeoptions)
        
        x0i = np.zeros((self.model.nvar,1))
        self.x0 = np.transpose(np.tile(x0i, (1, self.model.N)))
    
    def compute_action(self, state, reference_path):

        problem = {"x0": self.x0, "xinit": state}

        next_path_points = extract_next_path_points(reference_path, state[:3], self.model.N)
        problem["all_parameters"] = np.reshape(next_path_points, (self.model.npar*self.model.N,1))

        output, exitflag, info = self.solver.solve(problem)

        # Make sure the solver has exited properly.
        # assert exitflag == 1, "bad exitflag"
        sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
            .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((np.max(self.model.nvar), self.model.N))
        for i in range(self.model.N):
            temp[:, i] = output['x{0:02d}'.format(i+1)]
        problem["x0"] = temp
        pred_u = temp[0:4, :]
        pred_x = temp[4:16, :]
        return pred_u[:,0]

