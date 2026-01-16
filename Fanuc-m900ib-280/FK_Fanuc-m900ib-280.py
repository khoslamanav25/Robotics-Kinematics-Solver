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
    [theta1, -90, 410, 0], 
    [theta2-90, 180, 1120, 0], 
    [theta2+theta3+180, 90, -250, 0], 
    [theta4, -90, 0, -1285], 
    [theta5, 90, 0, 0], 
    [theta6+180, 180, 0, -300] 
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
'''
    Sample Output: 
    [[ 8.59075284e-01  1.83488889e-01  4.77829973e-01  1.99536225e+02]
    [ 1.30692715e-01 -9.81226027e-01  1.41827001e-01 -3.53045862e+02]
    [ 4.94882885e-01 -5.93911746e-02 -8.66927689e-01  3.76770517e+02]
    [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
'''

print("Position (x, y, z):", position)
'''
    Sample Output: 
    [ 199.53622485 -353.04586249  376.77051744]
'''