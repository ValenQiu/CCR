import numpy as np
import pandas as pd
import math
from scipy.optimize import fsolve

# p, R, q, w = [], [], [], []
j, n, m, v, u = None, None, None, None, None
vs, us, vt, ut, qt = None, None, None, None, None
wt, vst, ust, vh, uh = None, None, None, None, None
vsh, ush, qh, wh, nLL, mLL = None, None, None, None, None, None
x, z, X, Y, Z = None, None, None, None, None
y = [0, 0, 0] *3
Tt1, Tt2, rt1, rt2 = None, None, None, None
ns, e_dot, e, C = None, None, None, None
# Make vars available in whole program


def hat(y_in):
    y = [[0, -y_in[3], y_in[2]],
         [y_in[3], 0, -y_in[1]],
         [y_in[2], y_in[1], 0]]
    return y


def Backstepping_control_reach_90degrees():  # Back_stepping_control_reach_90degrees
    # parameters
    L = 0.5  # Length (before strain)
    N = 50  # Spatial resolution
    E = 207e9  # Young's modulus
    r = 0.001  # Cross-section radius
    rt1 = [0.02, 0, 0]
    rt2 = [-0.02, 0, 0]
    rho = 8000  # Density
    g = [-9.81, 0, 0]  # Gravity
    Bse = np.zeros(3)  # Material damping coefficients - shear and extension
    Bbt = 1e-2 * np.eye(3)  # Material damping coefficients - bending and torsion
    C = 0.1 * np.eye(3)
    dt = 0.04  # Time step
    alpha = -0.2  # BDF-alpha parameter
    STEPS = 202  # Number of timesteps to completion
    vstar = [0, 0, 1]  # Value of v when static and absent loading
    ustar = [0, 0, 0]  # Precurvature
    vsstar = [0, 0, 0]
    usstar = [0, 0, 0]

    # Boundary Conditions
    p = np.zeros((STEPS, N), dtype=object)
    R = np.zeros((STEPS, N), dtype=object)
    q = np.zeros((STEPS, N), dtype=object)
    w = np.zeros((STEPS, N), dtype=object)
    # global p, R, q, w
    for i in range(STEPS):    # from 1 to STEPS
        p[i][0] = np.array([0, 0, 0])
        R[i][0] = np.eye(3)
        q[i][0] = np.array([0, 0, 0])
        w[i][0] = np.array([0, 0, 0])
    # print(p)

    nL = [0, 0, 0]
    mL = [0, 0, 0]

    # Dependent Parameter Calculations
    A = math.pi*r**2    # Cross-sectional area
    J = np.diag([math.pi*r**4, math.pi*r**4/4, math.pi*r**4/2])  # Inertia
    G = 80e9    # shear modulus
    Kse = np.matrix(np.diag([G*A, G*A, E*A]))   # Stiffness matrix - shear and extension
    Kbt = np.diag([E*J[0, 0], E*J[1, 1], G*J[2, 2]])    # Stiffness matrix - bending and torsion
    ds = L/(N-1)    # Grid distance (before strain)
    c0 = (1.5 + alpha) / ( dt*(1+alpha) )   # BDF-alpha coefficients
    c1 = -2 / dt
    c2 = (0.5 + alpha) / (dt * (1 + alpha))
    d1 = alpha / (1 + alpha)

    # Main Simulation
    i = 1
    # fsolve(staticIVP, np.zeros(6,1))
    # applyStaticBDFalpha()

    f = 1
    global Tt1, Tt2
    for i in range(2, STEPS):
        if i < 3:
            Tt1 = 0
            Tt2 = 0
        else:
            # TT = find_tension()
            TT = 0
            if TT > 0:
                Tt1 = TT
            else:
                Tt1 = 0

        # fsolve(dynamicIVP, [n])
        # applyDynamicBDFalpha()
        # visualize()

        if i < 3:
            e = 0
            e_dot = 0
            TT = 0

    i = 1
    YY = np.array([(i-2)*dt, TT, p[i-1][N-1][0], p[i-1][N-1][2], e[0], e_dot[0]])
    f = f+1

        # Function Definitions
        def find_tension():
            t = (i-2)*dt

            P_desire = [0.25 * (1 - math.exp(-2 * t)), 0, 0]
            Pd_desire = [0.25 * (2 * math.exp(-2 * t)), 0, 0]
            Pdd_desire = [0.25 * (-4 * math.exp(-2 * t)), 0, 0]

            p_dot1 = R[i-1][N-1]*(hat(u[i-1][N-1])*rt1+v[i-1][N-1])
            p_ddot1 = R[i-1][N-1]*(hat(u[i-1][N-1])*(hat(u[i-1][N-1])*rt1+v[i-1][N-1])+hat(us[i-1][N-1])*rt1+vs[i-1,N-1])

            alph1 = (hat(p_dot1) * hat(p_dot1) * p_ddot1) / (norm(p_dot1) ^ 3)
            alph11 = (p_dot1) / (norm(p_dot1))

            alphM = alph1 + alph11

            ac = (ns + rho*A*g - R[i-1][N-1]*C*q[i-1][N-1]*abs(q[i-1][N-1]))/(rho*A)
            bc = -(alphM) / (rho * A)

            X_end = p
            {i - 1, N}
            Xd_end = (R[i-1][N] * q[i-1][N])

            alpa1 = 64.7
            alpa2 = 10

            Z1 = P_desire - X_end
            Z2 = Xd_end - Pd_desire - alpa1 * Z1

            TT = bc/(Z1 - ac - alpa1 * (Z2 + alpa1 * Z1) - alpa2 * Z2 + Pdd_desire)

            if TT > 87:
                TT = 87 # fsolve can be solved in this condition

            e = P_desire - X_end
            e_dot = Pd_desire - Xd_end

            return TT

        def staticIVP(G):
            n[i][1] = G[]


if __name__ == '__main__':
    Backstepping_control_reach_90degrees()
