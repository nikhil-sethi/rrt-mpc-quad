from trajectory import TrajectoryGenerator, fact, pow
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from cvxopt import solvers, matrix
from itertools import product

wps = [
	[0,2,4,7],
	[0,4,2,7]
]

dims = np.array(wps).shape[0] # dimensions
waypoints = np.array(wps).shape[1] 
segments = waypoints-1 # segments
order = 2 # minimum acceleration

t = [0,1,2,4]
T = t[-1]

# decision variables
X = [10]*segments*2*order # (segments)(2n)

# CONSTRAINTS
# Ax=b

# endpoint constraints
# order*2

A_ep = []

# first waypoint
m=0
for n in range(order):
	A_ep_i = [0]*2*order*m
	A_ep_i.extend([fact(j,n)*pow(t[m], j-n) for j in range(2*order)])	
	A_ep_i.extend([0]*2*order*(segments-m-1))
	A_ep.append(A_ep_i)

# last waypoint
m = segments
for n in range(order):
	A_ep_i = [0]*2*order*(m-1)
	A_ep_i.extend([fact(j,n)*pow(t[m], j-n) for j in range(2*order)])	
	A_ep_i.extend([0]*2*order*(segments-m))
	A_ep.append(A_ep_i)


# A_ep = [
# 	[t[0]**0,t[0]**1,t[0]**2,t[0]**3,0,0,0,0], # position wp 0
# 	[t[1]**0,t[1]**1,t[1]**2,t[1]**3,0,0,0,0], # position wp 1
# 	[0,t[0]**0,2*t[0]**1,3*t[0]**2,0,0,0,0],  # velocity wp 0
# 	[0,t[1]**0,2*t[1]**1,3*t[1]**2, 0, -(t[1]**0), -(2*t[1]**1), -(3*t[1]**2)], # velocity wp 1
# 	[0,0, 2*t[1]**0,6*t[1]**1, 0, 0, -(2*t[1]**0), -(6*t[1]**1)], # acc wp 1
# 	[0,0,0,0, t[1]**0, t[1]**1,   t[1]**2,   t[1]**3], # position wp 1
# 	[0,0,0,0, t[2]**0, t[2]**1,   t[2]**2,   t[2]**3], # position wp 2
# 	[0,0,0,0, 0, 	   t[2]**0, 2*t[2]**1, 3*t[2]**2],   # velocity wp 2
# ]




# continuity constraints,
# (order+1)*(segments-1) // if segments = 1, then (order-1)*1


A_con = []
for m in range(1,segments):
	for n in range(order+1):
		tpoly = [fact(j,n)*pow(t[m], j-n) for j in range(2*order)]
		if n==0:
			A_con_i = [0]*2*order*(m-1)
			# add 2 position constraints at same time index
			A_con_i.extend(tpoly + [0]*2*order)	# end position of prev poly
			A_con_i.extend([0]*2*order*(segments-m-1))
			A_con.append(A_con_i)

			A_con_i = [0]*2*order*(m-1)
			# add 2 position constraints at same time index
			A_con_i.extend([0]*2*order + tpoly)	# end position of prev poly
			A_con_i.extend([0]*2*order*(segments-m-1))
			A_con.append(A_con_i)

		else: # add order-1 continuity constraints
			A_con_i = [0]*2*order*(m-1)
			A_con_i.extend(tpoly + [-a for a in tpoly])
			A_con_i.extend([0]*2*order*(segments-m-1))
			A_con.append(A_con_i)
				
# b_con = [0]*(segments-1)


# combine endpoint and continuity constraints

A = A_ep + A_con

# function definitiion
def get_H_1seg(T):
	H = np.array([
		np.zeros(4),
		np.zeros(4),
		[0,0,4*T, 6*T**2],
		[0,0,6*T**2, 12*T**3]
	])
	return H

def get_H_1seg(T, n):
	"""Quadratic Cost matrix for integration of a squared n degree polynomial"""
	H_seg = np.zeros((2*n, 2*n))

	# The derivative coeffecients that come about after differentiating the polynomial n times
	diff = [fact(j,n) for j in range(n, 2*n)]
	# When sqaured and integrated the polymial will be quadratic and will have pairwise permutations of these coefficients
	# These coefficients will come up in the matrix afterwards 
	coeff_prods = np.prod(list(product(diff,repeat=2)),-1).reshape(n,n)

	# the powers to which the time endpoint will be raised. This comes from the result of integrating the sqaure of the
	# n times differentiated polynomial.
	time_powers = np.sum(np.meshgrid(np.arange(n), np.arange(n)),0)+1 
	time_poly = T**time_powers/time_powers # this result comes because of 2n-1 order integration
	H_seg[n:,n:] = coeff_prods*time_poly
	
	return H_seg

H = np.zeros((2*order*segments, 2*order*segments))
for m in range(segments):
	H[2*order*m:2*order*(m+1), 2*order*m:2*order*(m+1)] = get_H_1seg(t[m+1], n=order)

f = np.zeros(len(X))

traj = []

for l in range(dims):
	# b = [wps[l][0], wps[l][1], 0, 0, 0,wps[l][1], wps[l][2], 0] # endpoint RHS
	b_ep = [wps[l][0]] + [0]*(order-1) + [wps[l][-1]] + [0]*(order-1)
	b_con = []
	for i in range(1, segments):
		b_con += [wps[l][i]]*2 + [0]*order 
	b = b_ep + b_con	# continuity RHS

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