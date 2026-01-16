import numpy as np

def dh_matrix(alpha, a, d, theta):
    """Create the Denavit-Hartenberg transformation matrix."""
    alpha = np.deg2rad(alpha)
    theta = np.deg2rad(theta)
    
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def clamp(value, min_value, max_value):
    """Clamp the input value to the specified range."""
    return max(min_value, min(value, max_value))

def inverse_kinematics(P, DhParam):
    """Calculate the inverse kinematics solution."""
    a = [param[2] for param in DhParam]
    alpha = [param[1] for param in DhParam]
    d = [param[3] for param in DhParam]

    Px, Py, Pz = P[0], P[1], P[2]

    # Calculate Joint 1
    J1_1 = np.arctan2(Py, Px)
    J1_2 = np.arctan2(Py, Px) + np.pi
    J1_deg = [np.rad2deg(J1_1), np.rad2deg(J1_2)]

    # Select the first solution for Joint 1
    T01 = dh_matrix(alpha[0], a[0], d[0], J1_deg[0])

    # Calculate Wrist Center
    P46 = d[5] * np.array([0, 0, 1])
    P06 = np.array([Px, Py, Pz])
    P04 = P06 - P46  # Wrist Center (Kinematic Decoupling)

    # Calculate Joint 3
    T12 = dh_matrix(alpha[1], a[1], d[1], 0)  # theta2 = 0 for solution
    TempT02 = np.dot(T01, T12)
    P02 = TempT02[:3, 3]
    P24 = P04 - P02

    L1 = np.sqrt(a[3]**2 + d[3]**2)
    Gamma = np.arctan2(d[3], a[3])

    # Clamp the value for Phi calculation
    Phi_value = clamp((a[2]**2 + L1**2 - np.linalg.norm(P24)**2) / (2 * a[2] * L1), -1.0, 1.0)
    Phi = np.arccos(Phi_value)
    n = 1  # or -1 depending on your desired solution

    J3_1 = n * (np.pi - (Phi - Gamma))
    J3_2 = n * (np.pi - (Phi + Gamma))
    J3_deg = [np.rad2deg(J3_1), np.rad2deg(J3_2)]

    # Select the correct solution for Joint 3
    T23 = dh_matrix(alpha[2], a[2], d[2], J3_deg[1])

    # Calculate Joint 2
    P24_1 = np.dot(np.linalg.inv(T01[:3, :3]), P24)
    Beta1 = np.arctan2(P24_1[2], P24_1[0])

    # Clamp the value for Beta2 calculation
    Beta2_value = clamp((a[2]**2 + np.linalg.norm(P24)**2 - L1**2) / (2 * a[2] * np.linalg.norm(P24)), -1.0, 1.0)
    Beta2 = np.arccos(Beta2_value)

    J2_1 = n * (Beta1 + Beta2)
    J2_2 = n * (Beta1 - Beta2)
    J2_deg = [np.rad2deg(J2_1), np.rad2deg(J2_2)]

    T12 = dh_matrix(alpha[1], a[1], d[1], J2_deg[0])

    # Calculate Joint 5
    T34 = dh_matrix(alpha[3], a[3], d[3], 0)
    T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)

    # Clamp the value for J5 calculation
    J5_value = clamp(np.dot(T04[:3, 2], np.array([0, 0, 1])), -1.0, 1.0)
    J5_1 = np.arccos(J5_value)
    J5_deg = [np.rad2deg(J5_1)]

    # Calculate Joint 4
    R46 = np.dot(np.linalg.inv(T04[:3, :3]), np.identity(3))
    J4_1 = np.arctan2(R46[1, 2], R46[0, 2])
    J4_deg = [np.rad2deg(J4_1)]

    # Calculate Joint 6
    J6_1 = np.arctan2(R46[2, 1], -R46[2, 0])
    J6_deg = [np.rad2deg((np.pi / 2) - J6_1)]

    JointAngle = [J1_deg[0], J2_deg[0], J3_deg[1], J4_deg[0], J5_deg[0], J6_deg[0]]
    return JointAngle

# Example usage:
theta1 = 0  # Replace with your specific theta1 value
theta2 = 0  # Replace with your specific theta2 value
theta3 = 0  # Replace with your specific theta3 value
theta4 = 0  # Replace with your specific theta4 value
theta5 = 0  # Replace with your specific theta5 value
theta6 = 0  # Replace with your specific theta6 value

dh_params = [
    [theta1, -90, 0, 290],
    [theta2 - 90, 0, 270, 0], 
    [theta3 + 180, 90, -70, 0], 
    [theta4, -90, 0, 302],
    [theta5, 90, 0, 0], 
    [theta6, 0, 0, 72]
]

P = np.array([410.31972254, 74.55499564, 482.56498211])

joint_angles = inverse_kinematics(P, dh_params)
print("Joint Angles:", joint_angles)
