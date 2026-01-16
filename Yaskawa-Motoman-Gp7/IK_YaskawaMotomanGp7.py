import numpy as np 
import math 

# Target Position (Origin to T)
OpT = np.array([437.05, -336.44, -30.19])

# Target Rotation 
OpR = np.array([
    [0.859, 0.183, 0.478],
    [0.131, -0.981, 0.142],
    [0.495, -0.0594, -0.867]
])

# DH parameters: theta, alpha, a, d (alpha in degrees)
dh_params = [
    [10, 0, 0, 0], # Base --> S
    [10, 90, 0, 445], # S --> L
    [10, 0, 40, 0], # L --> U (negative?)
    [10, 90, 0, 400], # U --> R (a or d?)
    [10, -90, 0, 80], # R --> B (a or d?)
    [10, 90, 0, 0] # B --> T
]

def cos(degrees):
    return math.cos(math.radians(degrees))

def sin(degrees):
    return math.sin(math.radians(degrees))

# S, L, U, R, B, T
t_matrices = []

for joint_param in dh_params:
    theta = joint_param[0]
    alpha = joint_param[1]
    a = joint_param[2]
    d = joint_param[3]

    t_matrix = np.array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

    t_matrices.append(t_matrix)

final_matrix = np.eye(4)
for matrix in t_matrices[:4]:
    final_matrix = np.dot(final_matrix, matrix)

# Position B from Origin
OpB = OpT - 80 * OpR[:, 2]
Xb, Yb, Zb = OpB

