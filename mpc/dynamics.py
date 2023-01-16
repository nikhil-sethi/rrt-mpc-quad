import casadi

def continuous_nonlinear_dynamics_3d(x, u):
    # Model based on the slides from Robot Dynamics & Control
    # Return the derivatives of the state
    m = 0.027 # [kg]
    g = 9.8 # [m/s^2]
    Ixx = 1.4e-5 # [kg/m^2]
    Iyy = 1.4e-5 # [kg/m^2]
    Izz = 2.17e-5 # [kg/m^2]

    L = 0.0397 # [m]
    k_F = 3.16e-10 # [N/rpm^2]
    k_M = 7.94e-12 # [N/rpm^2]
    # rpm to force:
    F1 = k_F * u[3]**2
    F2 = k_F * u[0]**2
    F3 = k_F * u[1]**2
    F4 = k_F * u[2]**2
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
    x_dot_9 = -((-Iyy + Izz) / Ixx) * x[7] * x[8] + 1/Ixx * u2
    x_dot_10 = -((Ixx - Izz) / Iyy) * x[6] * x[8] + 1/Iyy * u3
    x_dot_11 = -((-Ixx + Iyy) / Izz) * x[6] * x[7] + 1/Izz * u4


    return casadi.vertcat(x_dot_0, x_dot_1, x_dot_2, x_dot_3, 
                          x_dot_4, x_dot_5, x_dot_6, x_dot_7,
                          x_dot_8, x_dot_9, x_dot_10, x_dot_11)

def dynamics_base_aviary(x,u):
    # Model based on the Dynamics given in BaseAviary in gym pybullet drones
    # Return the derivatives of the state
    m = 0.027 # [kg]
    g = 9.8 # [m/s^2]
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
    forces = (F1**2 + F2**2 + F3**2 + F4**2) * k_F

    c1 = casadi.cos(x[6])
    s1 = casadi.sin(x[6])
    c2 = casadi.cos(x[7])
    s2 = casadi.sin(x[7])
    c3 = casadi.cos(x[8])
    s3 = casadi.sin(x[8])

    mat31 = -s2
    mat32 = c2 * s3
    mat33 = c2 * c3

    FW1 = forces * mat31
    FW2 = forces * mat32
    FW3 = forces * mat33 - g
    x3_dot = FW1 / m
    x4_dot = FW2 / m
    x5_dot = FW3 / m
    ZT1 = k_M * u[0] ** 2
    ZT2 = k_M * u[1] ** 2
    ZT3 = k_M * u[2] ** 2
    ZT4 = k_M * u[3] ** 2
    z_torque = -ZT1 + ZT2 - ZT3 + ZT4
    x_torque = (F1 + F2 - F3 - F4) * L/casadi.sqrt(2)
    y_torque = (-F1 + F2 + F3 - F4) * L / casadi.sqrt(2)
    T1 = x_torque - (-z_torque*Iyy*x[10] + y_torque*Izz*x[11])
    T2 = y_torque - (-x_torque * Izz * x[11] + z_torque * Ixx * x[9])
    T3 = z_torque - (x_torque * Iyy * x[10] - y_torque * Ixx * x[9])
    x9_dot = T1/Ixx
    x10_dot = T2/Iyy
    x11_dot = T3/Izz
    return casadi.vertcat(x[3], x[4], x[5], x3_dot,
                          x4_dot, x5_dot, x[9], x[10],
                          x[11], x9_dot, x10_dot, x11_dot)