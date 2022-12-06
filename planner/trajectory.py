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
    [0, 0.5, 0, 5], # vel
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

# f = 1

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

plt.plot(plan_x, plan_y, 'r-')

t_id = 1

coeffs_x = get_coeffs(x, k, t_id, t[t_id:t_id+2])
coeffs_y = get_coeffs(y, k, t_id, t[t_id:t_id+2])

# print(coeffs)
times = np.linspace(t[t_id],t[t_id+1],20)
plan_x = traj(times, coeffs_x)
plan_y = traj(times, coeffs_y)

plt.plot(plan_x, plan_y, 'r-')


plt.plot(x[0][:t_id+2], y[0][:t_id+2], 'b-')
# plt.plot(x[:t_id+2], y[:t_id+2], 'b.')

plt.show()