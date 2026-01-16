import numpy as np
import math

theta1 = float(input("Theta 1 (Degrees): "))
theta2 = float(input("Theta 2 (Degrees): "))
theta3 = float(input("Theta 3 (Degrees): "))
theta4 = float(input("Theta 4 (Degrees): "))
theta5 = float(input("Theta 5 (Degrees): "))
theta6 = float(input("Theta 6 (Degrees): "))

# DH parameters: theta, alpha, a, d (theta, alpha in degrees)
dh_params = [
    [theta1, -90, 0, 290],
    [theta2-90, 0, 270, 0], 
    [theta3+180, 90, -70, 0], 
    [theta4, -90, 0, 302],
    [theta5, 90, 0, 0], 
    [theta6, 0, 0, 72] 
]

t_matrices = []

def cos(degrees):
    return math.cos(math.radians(degrees))

def sin(degrees):
    return math.sin(math.radians(degrees))

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
for matrix in t_matrices:
    final_matrix = np.dot(final_matrix, matrix)

position = final_matrix[:3, 3]

print("Final transformation matrix:")
print(final_matrix)

print("Position (x, y, z):", position)