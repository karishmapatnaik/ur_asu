import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

# UR5e DH parameters
dh_params = [
    (0,  0.1625,  0,     np.pi/2),  
    (0,  0,      -0.425,  0),       
    (0,  0,      -0.3922, 0),       
    (0,  0.1333,  0,     np.pi/2),  
    (0,  0.0997,  0,    -np.pi/2),  
    (0,  0.0996,  0,     0)
]

def rpy_to_matrix(rpy):
    return R.from_euler('xyz', rpy, degrees=True).as_matrix()

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ])

def forward_kinematics(dh_params, joint_angles):
    T = np.eye(4)
    for i, (theta, d, a, alpha) in enumerate(dh_params):
        T_i = dh_transform(joint_angles[i] + theta, d, a, alpha)
        T = np.dot(T, T_i)
    return T

def ik_objective(q, target_pose):
    T_fk = forward_kinematics(dh_params, q)
    pos_error = np.linalg.norm(T_fk[:3, 3] - target_pose[:3, 3])
    rot_error = np.linalg.norm(T_fk[:3, :3] - target_pose[:3, :3])
    return 1.0 * pos_error + 0.1 * rot_error

def inverse_kinematics(position, rpy, q_guess=None):
    if q_guess is None:
        q_guess = np.radians([85, -80, 90, -90, -90, 90])

    target_pose = np.eye(4)
    target_pose[:3, 3] = np.array(position)
    target_pose[:3, :3] = rpy_to_matrix(rpy)

    joint_bounds = [
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi)
    ]

    result = minimize(ik_objective, q_guess, args=(target_pose,), method='L-BFGS-B', bounds=joint_bounds)

    if result.success:
        return result.x
    else:
        raise ValueError("❌ IK Solution Not Found!")
