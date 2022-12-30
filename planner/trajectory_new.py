from trajectory import TrajectoryGenerator, fact, pow
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from cvxopt import solvers, matrix
from itertools import product

wps = [
    [0,1,4],
    [0,4,2]
]

dims = np.array(wps).shape[0] # dimensions
waypoints = np.array(wps).shape[1] 
segments = waypoints-1 # segments
order = 2 # minimum acceleration

t = [0,1,2]
T = t[-1]

# decision variables
X = [10]*segments*2*order # (segments)(2n)

# CONSTRAINTS
# Ax=b

A = [
	[t[0]**0,t[0]**1,t[0]**2,t[0]**3,0,0,0,0], # position wp 0
	[t[1]**0,t[1]**1,t[1]**2,t[1]**3,0,0,0,0], # position wp 1
	[0,t[0]**0,2*t[0]**1,3*t[0]**2,0,0,0,0],  # velocity wp 0
	[0,t[1]**0,2*t[1]**1,3*t[1]**2, 0, -(t[1]**0), -(2*t[1]**1), -(3*t[1]**2)], # velocity wp 1
	[0,0, 2*t[1]**0,6*t[1]**1, 0, 0, -(2*t[1]**0), -(6*t[1]**1)], # acc wp 1
	[0,0,0,0, t[1]**0, t[1]**1,   t[1]**2,   t[1]**3], # position wp 1
	[0,0,0,0, t[2]**0, t[2]**1,   t[2]**2,   t[2]**3], # position wp 2
	[0,0,0,0, 0, 	   t[2]**0, 2*t[2]**1, 3*t[2]**2],   # velocity wp 2
]


# function definitiion
def get_H_1seg(T):
    H = np.array([
        np.zeros(4),
        np.zeros(4),
        [0,0,4*T, 6*T**2],
        [0,0,6*T**2, 12*T**3]
    ])
    return H


H = np.zeros((2*order*segments, 2*order*segments))
for m in range(segments):
    H[2*order*m:2*order*(m+1), 2*order*m:2*order*(m+1)] = get_H_1seg(t[m+1])

f = np.zeros(len(X))

traj = []

for l in range(dims):
    b = [wps[l][0], wps[l][1], 0, 0, 0,wps[l][1], wps[l][2], 0] # endpoint RHS

    sol = solvers.qp(P = matrix(H), q=matrix(f), A=matrix(np.array(A), tc='d'), b=matrix(b, tc='d'))
    print(sol["x"])
    print("cost = ", sol["primal objective"])
    C = np.expand_dims((np.array(sol['x']).reshape(segments,2*order)),-1) # coefficients

    d = 40 # discretisation
    pts_per_poly = d//segments
    tvec = np.linspace(t[:-1],t[1:], pts_per_poly).T.reshape(segments, pts_per_poly,1)
    pvec = np.tile(tvec, 2*order) ** np.arange(2*order) # (d/l x order)

    traj.append((pvec@C).flatten())
traj = np.array(traj).T


# PLOTTING
# 1D
# tvec = tvec.reshape(l, pts_per_poly*segments).T
# plt.plot(t, wps[0], 'b-')
# plt.plot(tvec, traj, 'r-')

# 2D
plt.plot(wps[0],wps[1],'b-')
plt.plot(traj[:,0],traj[:,1],'r-')
plt.show()