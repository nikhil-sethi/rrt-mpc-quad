import numpy as np
import matplotlib.pyplot as plt
# from .. import utils

def fact(j, i):
    """factorial of j upto i. Used for derivatives"""
    if i==0:
        return 1
    return j*fact(j-1, i-1)

def pow(n, k):
    if k<0:
        return 1
    return n**k


class TrajectoryGen():
    def __init__(self, n, wps) -> None:
        self.wps = np.array(wps) # time constrained waypoints
        self.t = self.wps[-1,:]

        self.l  = self.wps.shape[0]-1 # number of dimensions
        self.m = self.wps.shape[1] # number of waypoints
        self.n = n # order of control/number of derivatives

        self.path_wts = None
        self.constraints = np.zeros((self.l, self.m, self.n)) # a consise tensor with all n-1 derivative

    def get_wts(self):
        """returns distance based weights of the path"""
        pass

    def to_constraints(self, X):
        """
        X: the decision variable vector (a linear vector representation for optimization)
        
        constraints: fully formulated boundary conditions at all points
        axis 0: dimension (x,y,z)
        axis 1: number of points (p1,p2,p3)
        axis 2: number of derivatives (p',p'',p''')

        ####
        example (2,4,4) a 2D minimum snap trajectory for 4 points
        X = [x1',x1'',x1''', x2',x2'',x2''', y1',y1'',y1''', y2',y2'',y2'''] // length = l(m-2)(n-1)

        constraints =
            [[[x0. 0. 0. 0.] // at rest
              [x1. x1',x1'',x1''']
              [x2. x2',x2'',x2''']
              [x3. 0. 0. 0.]] // at rest
  
             [[y0. 0. 0. 0.] // at rest
              [y1. y1',y1'',y1''']
              [y2. y2',y2'',y2''']
              [y3. 0. 0. 0.]] // at rest


        """

        wps = self.wps
        assert len(X) == self.l*(self.m-2)*(self.n-1), "Insufficient or extra number of decision variables"

        self.constraints[:,[0,-1],0] = wps[:-1,[0,-1]]    # first point is at rest, all n-1 derivatives are zero
        for i in range(self.l): # for all dimensions x,y,z...
            for j in range(self.m - 2): # for all points except first and last
                idx = (i*(self.m-2) + j)*(self.n-1) # this conversion is needed becuase the decision variable is a 1D list, but constraints is 3D
                self.constraints[i,j+1,:] = [wps[i][j+1]] + X[idx:idx + self.n-1]  # updates the n-1 derivatives for all points except first and last
    @property
    def A(self):
        """The boundary condition tensor"""

        # the next line creates the end conditions for all possible derivatives, dimensions and waypoints. (vector on the left side from slides)
        _A = np.array([np.insert(self.constraints[i][1:,:], range(self.n), self.constraints[i][:-1,:], axis=1) for i in range(self.l)])

        return _A.reshape(self.l, self.m-1, 2*self.n, 1)  # reshaped to make it cleaner

    @property
    def M(self):
        """Returns the polynomial tensor"""
        _M = np.empty((len(self.t)-1, 2*self.n, 2*self.n)) 
        for i in range(self.n):
            for j in range(2*self.n):
                _M[:,2*i, j] = fact(j,i) * pow(self.t[:-1], j-i)
                _M[:,2*i+1, j] = fact(j,i) * pow(self.t[1:], j-i)
        return _M
    
    def generate(self, d = 100):
        C = np.linalg.inv(self.M) @ self.A  # (l x n x m-1) The coefficients for each dimension and each polynomial
        
        pts_per_poly = d//(self.m-1)

        # the next line discretizes the trajectory based on the nth order polynomial
        T = np.tile(np.linspace(self.t[:-1],self.t[1:], pts_per_poly).T.reshape(self.m-1, pts_per_poly,1), 2*self.n) ** np.arange(2*self.n) # (l x d/l x n)
        
        return (T@C).squeeze(axis=-1).transpose(0,2,1).transpose(2,1,0) # (l x d/l x m-1)  

    def plot(self, plan):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(self.wps[0],self.wps[1],self.wps[2],'b-')
        for p_i in plan:
            ax.plot(p_i[:,0],p_i[:,1],p_i[:,2], 'r-')
        # plt.show()

# time bound waypoints
# (x,y,t)   2D
waypoints = np.array([
    [0,2,4,7], # all xs
    [0,4,2,7], # all ys
    [0,3,2,2], # all zs
    [0,3,5,7]  # all ts
])


# l: number of dimensions
# m: number of waypoints
# n: number of derivatives/order

# decision vector
# (n-1)(m-2)l variables without time optimization 

X = [
    # x1_d, x1_dd, x1_ddd, x2_d, x2_dd ... m-1,
    # y1_d, y1_dd, y1_ddd, y2_d, y2_dd ... m-1,
    # z1_d, z1_dd, z1_ddd, z2_d, z2_dd ... m-1,
    1,0,0, 1,0,0, 
    1,0,0, 1,0,0, 
    1,0,0, 0.2,0,0, 
] # the optimization vector. can indlude times as well.
# during optimization, velocity and acceleration constraints will be imposed on the magnitudes

order = 4 # min snap

tgen = TrajectoryGen(n = order, wps=waypoints)

tgen.to_constraints(X)
traj = tgen.generate(d = 100) # numpy array of 30x2 points of the trajectory

tgen.plot(traj)

plt.show()