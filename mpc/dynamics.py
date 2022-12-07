import casadi

def continuous_nonlinear_dynamics_3d(x, u):
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
                          x_dot_8, x_dot_9, x_dot_10, x_dot_11) 
