import numpy as np

# ---------------------------
# Robot Parameters
# ---------------------------
L1 = 0.0275   # Coxa length (m)
L2 = 0.055    # Femur length
L3 = 0.08185  # Tibia length
STEP_SIZE = 0.010  # 10 mm forward

# Base positions of legs
base_positions = np.array([
    [0.14757, -0.04705, 0.10935],
    [0.08707, -0.04705, 0.10935],
    [0.02657, -0.04705, 0.10935],
    [-0.03393, -0.04705, 0.10935],
    [0.14757,  0.03994, 0.10935],
    [0.08707,  0.03994, 0.10935],
    [0.02657,  0.03994, 0.10935],
    [-0.03393, 0.03994, 0.10935]
])

# Initial foot positions
foot_init = np.array([
    [0.160, -0.060, 0.030],
    [0.100, -0.060, 0.030],
    [0.040, -0.060, 0.030],
    [-0.020, -0.060, 0.030],
    [0.160,  0.060, 0.030],
    [0.100,  0.060, 0.030],
    [0.040,  0.060, 0.030],
    [-0.020, 0.060, 0.030]
])

# ---------------------------
# IK Function
# ---------------------------
def leg_ik(foot_target, base_pos, L1, L2, L3):
    dx = foot_target[0] - base_pos[0]
    dy = foot_target[1] - base_pos[1]
    dz = foot_target[2] - base_pos[2]

    # Coxa
    v1 = np.arctan2(dy, dx)

    # Planar distances
    r = np.sqrt(dx**2 + dy**2) - L1
    s = dz
    D = np.sqrt(r**2 + s**2)
    D = min(D, L2 + L3 - 1e-6)

    # Tibia
    cos_v3 = (L2**2 + L3**2 - D**2) / (2*L2*L3)
    v3 = np.arccos(np.clip(cos_v3, -1, 1)) - np.pi/2

    # Femur
    alpha = np.arctan2(s, r)
    beta  = np.arccos(np.clip((L2**2 + D**2 - L3**2)/(2*L2*D), -1, 1))
    v2 = alpha + beta

    return v1, v2, v3
