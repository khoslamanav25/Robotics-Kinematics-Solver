import numpy as np
import math

# Function to calculate forward kinematics using DH parameters
def forward_kinematics(dh_params):
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

    return final_matrix

# Inverse kinematics function
def inverse_kinematics(T, dh_params):
    # Extract the position from the transformation matrix
    px = T[0, 3]
    py = T[1, 3]
    pz = T[2, 3]
    
    # DH parameters
    a1 = dh_params[1][2]
    d1 = dh_params[0][3]
    a2 = dh_params[1][2]
    d4 = dh_params[3][3]
    
    # Calculate theta1
    theta1 = np.arctan2(py, px)
    
    # Calculate r and s
    r = np.sqrt(px**2 + py**2) - a1
    s = pz - d1
    
    # Calculate theta3 using geometric approach
    D = (r**2 + s**2 - a2**2 - d4**2) / (2 * a2 * d4)
    theta3 = np.arctan2(np.sqrt(1 - D**2), D)
    
    # Calculate theta2
    theta2 = np.arctan2(s, r) - np.arctan2(d4 * np.sin(theta3), a2 + d4 * np.cos(theta3))
    
    # Simplified calculation for theta4, theta5, theta6
    theta4, theta5, theta6 = 0, 0, 0
    
    return np.degrees([theta1, theta2, theta3, theta4, theta5, theta6])

# Example: Using given joint angles for forward kinematics
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

# Calculate forward kinematics
T = forward_kinematics(dh_params)

# Print final transformation matrix and position
print("Final transformation matrix:")
print(T)
print("Position (x, y, z):", T[:3, 3])

# Calculate inverse kinematics based on the transformation matrix
ik_angles = inverse_kinematics(T, dh_params)

print("Calculated joint angles from IK (Degrees):", ik_angles)
