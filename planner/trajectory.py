import numpy as np
import matplotlib.pyplot as plt


t0 = 0
t1 = 2
t2 = 3
tm = 5

x0 = 0
x1 = 2
x2 = 3
xm = 5

v = 1

def fact(j, i):
    if i==0:
        return 1
    return j*fact(j-1, i-1)

def pow(n, k):
    if k<0:
        return 1
    return n**k

def traj(t, coeffs):
    y = 0
    for i in range(len(coeffs)):
        y += coeffs[i]*(t**i)
    return y

# waypont conditions
t = [0, 2, 5, 7]
# x = [0, 2, 3, 5]   # position
# x_1 = [5, 5, 5, 5]
# x_2 = [0, 0, 0, 0]
# x_3 = [0, 0, 0, 0] # 3rd derivative of position
k = 4 # order of control

x = np.array([
    [0, 2, 4, 5], # pos
    [0, 1, 0, 5], # vel
    [0, 0, 0, 0], # acc
    [0, 0, 0, 0]  # jerk
])
y = np.array([
    [0, 4, 2, 5], # pos
    [0, -0.5, 0, 5], # vel
    [0, 0, 0, 0], # acc
    [0, 0, 0, 0]  # jerk
])

def get_bounds(x, k, t):
    bounds = np.zeros((2*k, len(t)-1))
    print(x)
    for i in range(k):
        bounds[[2*i, 2*i+1],:] = [x[i,:-1], x[i,1:]]
    return bounds

def get_M(t_id, t):
    print(t)
    t_i = t[0]
    t_f = t[1]
    M = np.zeros((2*k, 2*k)) 
    for i in range(k):
        for j in range(2*k):
            M[2*i][j] = fact(j,i) * pow(t_i, j-i)
            M[2*i+1][j] = fact(j,i) * pow(t_f, j-i)
    return M

def get_M_vec(t):
    # t_i = t[0]
    # t_f = t[1]
    M = np.zeros((len(t)-1,2*k, 2*k)) 
    for i in range(k):
        for j in range(2*k):
            M[:,2*i, j] = fact(j,i) * pow(t[:-1], j-i)
            M[:,2*i+1, j] = fact(j,i) * pow(t[1:], j-i)
    return M


def get_coeffs(x, k, t_id, t):
    M = get_M(t_id, t)
    bounds = get_bounds(x[:,t_id:t_id+2], k, t)
    coeffs = np.linalg.inv(M) @ bounds
    return coeffs

t_id = 0 

coeffs_x = get_coeffs(x, k, t_id, t[t_id:t_id+2])
coeffs_y = get_coeffs(y, k, t_id, t[t_id:t_id+2])

# print(coeffs)
times = np.linspace(t[t_id],t[t_id+1],20)
plan_x = traj(times, coeffs_x)
plan_y = traj(times, coeffs_y)

# plt.plot(plan_x, plan_y, 'r-')

t_id = 1

coeffs_x = get_coeffs(x, k, t_id, t[t_id:t_id+2])
coeffs_y = get_coeffs(y, k, t_id, t[t_id:t_id+2])

# print(coeffs)
times = np.linspace(t[t_id],t[t_id+1],20)
plan_x = traj(times, coeffs_x)
plan_y = traj(times, coeffs_y)

# plt.plot(plan_x, plan_y, 'r-')


# plt.plot(x[0][:t_id+2], y[0][:t_id+2], 'b-')
# plt.plot(x[:t_id+2], y[:t_id+2], 'b.')

# plt.show()


class TrajectoryGen():
    def __init__(self, n, wps) -> None:
        self.wps = np.array(wps) # time constrained waypoints
        self.t = self.wps[-1,:]

        self.l  = self.wps.shape[0]-1 # number of dimensions
        self.m = self.wps.shape[1] # number of waypoints
        self.n = n # order of control/number of derivatives

        self.path_wts = None
        self.constraints = np.zeros((self.l, self.m, self.n)) 

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
            # self.constraints[i,0,:] = [wps[i][0]] + [0]*(self.n - 1)  
            # self.constraints[i,-1,:] = [wps[i][-1]] + [0]*(self.n - 1) # last point is at rest, all n-1 derivatives are zero
            
            for j in range(self.m - 2): # for all points except first and last
                idx = (i*(self.m-2) + j)*(self.n-1) # this conversion is needed becuase the decision variable is a 1D list, but constraints is 3D
                self.constraints[i,j+1,:] = [wps[i][j+1]] + X[idx:idx + self.n-1]  # updates the n-1 derivatives for all points except first and last
    @property
    def A(self):
        """The boundary condition tensor"""
        _A = np.array([np.insert(self.constraints[i][1:,:], range(self.n), self.constraints[i][:-1,:], axis=1) for i in range(self.l)])

        return _A.reshape(self.l, self.m-1, 2*self.n, 1)

    @property
    def M(self):
        _M = np.empty((len(self.t)-1, 2*self.n, 2*self.n)) 
        for i in range(self.n):
            for j in range(2*self.n):
                _M[:,2*i, j] = fact(j,i) * pow(self.t[:-1], j-i)
                _M[:,2*i+1, j] = fact(j,i) * pow(self.t[1:], j-i)
        return _M
    
    def generate(self, d = 100):
        # M = self.M
        # for A_i in self.A:
        C = np.linalg.inv(self.M) @ self.A

        T = np.tile(np.linspace(self.t[:-1],self.t[1:], d//(self.m-1)).T.reshape(self.m-1, d//(self.m-1),1), 2*self.n)** np.arange(2*self.n)
        print(C.shape, T.shape)
        return (T@C).squeeze(axis=-1).transpose(0,2,1).transpose(2,1,0) # (l x d x m-1) 
        # polys = np.array([np.poly1d(C[i].flatten()[::-1]) for i in range()])
        
        # return tfunc(np.linspace(0, self.t[-1], d))

    def plot(self, plan):
        for p_i in plan:
            plt.plot(p_i[:,0],p_i[:,1], 'r-')
        # plt.show()

# time bound waypoints
# (x,y,t)   2D
waypoints = np.array([
    [0,2,4,7], # all xs
    [0,4,2,7], # all ys
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
] # 

order = 4 # min snap

tgen = TrajectoryGen(n = order, wps=waypoints)
tgen.to_constraints(X)
# print(tgen.constraints)
# print(tgen.A)
# print(tgen.M[0])
# C = np.linalg.inv(tgen.M) @tgen.A
# print(C)

plt.plot(waypoints[0],waypoints[1],'b-')
plan = tgen.generate(d = 30) # numpy array of 30x2 points of the trajectory
print(plan)
tgen.plot(plan)
# times = np.linspace(waypoints[-1,0],waypoints[-1,1], 30)
# plt.plot(plan[0][:,0],plan[0][:,1], 'r-')
# # plt.plot(times,plan[0][:,0], 'r-')
# plt.plot(plan[1][:,0],plan[1][:,1], 'r-')
# plt.plot(np.linspace(0,waypoints[-1][-1], 30), plan, 'r.')
plt.show()