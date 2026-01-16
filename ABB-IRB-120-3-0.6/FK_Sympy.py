from sympy import *

theta1 = Symbol('θ1')
theta2 = Symbol('θ2')
theta3 = Symbol('θ3')
theta4 = Symbol('θ4')
theta5 = Symbol('θ5')
theta6 = Symbol('θ6')

dh_params = [
    [theta1, -90, 0, 290],
    [theta2-90, 0, 270, 0], 
    [theta3+180, 90, -70, 0], 
    [theta4, -90, 0, 302],
    [theta5, 90, 0, 0], 
    [theta6, 0, 0, 72] 
]

t_matrices = []

for joint_param in dh_params:
    theta = joint_param[0]
    alpha = joint_param[1]
    a = joint_param[2]
    d = joint_param[3]

    t_matrix = Matrix([
        [cos(rad(theta)), -sin(rad(theta))*cos(rad(alpha)), sin(rad(theta))*sin(rad(alpha)), a*cos(rad(theta))],
        [sin(rad(theta)), cos(rad(theta))*cos(rad(alpha)), -cos(rad(theta))*sin(rad(alpha)), a*sin(rad(theta))],
        [0, sin(rad(alpha)), cos(rad(alpha)), d],
        [0, 0, 0, 1]
    ])

    t_matrices.append(t_matrix)

final_matrix = eye(4)
for matrix in t_matrices:
    final_matrix = final_matrix * matrix

final_matrix * Matrix([0, 0, -72, 1])
print(final_matrix)
