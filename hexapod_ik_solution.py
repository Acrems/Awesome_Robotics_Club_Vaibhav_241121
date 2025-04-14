import numpy as np
import math

L1 = 5.0
L2 = 10.0
L3 = 15.0

def is_reachable(x, y, z, L1, L2, L3):
    r = math.hypot(x, y)
    eff_r=r-5
    total_dist=math.hypot(eff_r, z)
    min_reach=5
    max_reach=30
    return min_reach<=total_dist<= max_reach

def forward_kinematics(beta, gamma):
    R = 15*np.cos(beta+gamma)+10*np.cos(beta) +5
    Z = 15*np.sin(beta+gamma)+10*np.sin(beta)
    return np.array([R, Z])

def jacobian(beta, gamma):
    R_beta=-15*np.sin(beta+gamma)-10*np.sin(beta)
    R_gamma=-15*np.sin(beta+gamma)
    Z_beta=15*np.cos(beta+gamma)+10*np.cos(beta)
    Z_gamma=15*np.cos(beta+gamma)
    return np.array([
        [R_beta, R_gamma],
        [Z_beta, Z_gamma]
    ])

def inverse_kinematics_jacobian(r_target, z_target, beta_init=0.5, gamma_init=-1.0, tol=1e-6, max_iter=100):
    beta=beta_init
    gamma=gamma_init

    for i in range(max_iter):
        f = forward_kinematics(beta, gamma) - np.array([r_target, z_target])
        if np.linalg.norm(f) < tol:
            return beta, gamma

        J = jacobian(beta, gamma)
        delta = np.linalg.solve(J, f)

        beta-=delta[0]
        gamma-=delta[1]

    print("Did not converge — point may be unreachable.")
    return None, None


def inverse_kinematics(x, y, z):
    r_target = math.hypot(x, y)
    alpha = math.atan2(y, x)

    if is_reachable(x,y,z,L1,L2,L3):
        beta, gamma = inverse_kinematics_jacobian(r_target, z)
        print(f"SOLUTION : ")
        print(f"Alpha = {math.degrees(alpha):.2f} degrees")
        print(f"Beta = {math.degrees(beta):.2f} degrees")
        print(f"Gamma = {math.degrees(gamma):.2f} degrees")
    else:
        print("Unreachable")

def test_inverse_kinematics():
    inverse_kinematics(15,0,-5)
    inverse_kinematics(1,1,1)
    inverse_kinematics(5,10,14)
    inverse_kinematics(5,10,20)

test_inverse_kinematics()
