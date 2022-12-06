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
t = [0, 2, 3, 5]
# x = [0, 2, 3, 5]   # position
# x_1 = [5, 5, 5, 5]
# x_2 = [0, 0, 0, 0]
# x_3 = [0, 0, 0, 0] # 3rd derivative of position
k = 4 # order of control

x = np.array([
    [0, 2, 3, 5], # pos
    [0, 5, 5, 5], # vel
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
    t_i = t[t_id]
    t_f = t[t_id +1]
    M = np.zeros((2*k, 2*k)) 
    for i in range(k):
        for j in range(2*k):
            M[2*i][j] = fact(j,i) * pow(t_i, j-i)
            M[2*i+1][j] = fact(j,i) * pow(t_f, j-i)
    return M

# f = 1

def get_coeffs(x, k, t_id, t):
    M = get_M(t_id, t)
    bounds = get_bounds(x[:,:t_id+2], k, t)
    coeffs = np.linalg.inv(M) @ bounds
    return coeffs

t_id = 0

coeffs = get_coeffs(x, k, t_id, t[:t_id+2])

print(coeffs)
times = np.linspace(t[t_id],t[t_id+1],20)
plan = traj(times, coeffs)

plt.plot(times, plan, 'r.')
plt.show()