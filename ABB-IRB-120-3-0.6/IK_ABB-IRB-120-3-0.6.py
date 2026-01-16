import numpy as np

d1 = 290
a2 = 270
a3 = 70
d4 = 302
d6 = 72

P = np.array([410.31972254, 74.55499564, 482.56498211])
Px, Py, Pz = P[0], P[1], P[2]

XYZ = np.array([np.deg2rad(44.109), np.deg2rad(-61.789), np.deg2rad(144.109)])
X, Y, Z = XYZ[0], XYZ[1], XYZ[2]

Rx = np.array([
    [1, 0, 0],
    [0, np.cos(X), -np.sin(X)],
    [0, np.sin(X), np.cos(X)]
])

Ry = np.array([
    [np.cos(Y), 0, np.sin(Y)],
    [0, 1, 0],
    [-np.sin(Y), 0, np.cos(Y)]
])

Rz = np.array([
    [np.cos(Z), -np.sin(Z), 0],
    [np.sin(Z), np.cos(Z), 0],
    [0, 0, 1]
])

R = Rx @ Ry @ Rz

Pw = P - R @ np.array([0, 0, d6])
print(Pw)

theta1 = np.arctan2(Py, Px)

r = np.sqrt(Px**2 + Py**2) - a3
s = Pz - d1  

D = (r**2 + s**2 - a2**2 - d4**2) / (2 * a2 * d4)

if abs(D) > 1:
    raise ValueError("The position is out of the robot's reach, D is outside the valid range [-1, 1].")

theta3 = np.arctan2(np.sqrt(1 - D**2), D)

theta2 = np.arctan2(s, r) - np.arctan2(d4 * np.sin(theta3), a2 + d4 * np.cos(theta3))

theta4 = 0
theta5 = 0
theta6 = 0

theta1_deg = np.degrees(theta1)
theta2_deg = np.degrees(theta2)
theta3_deg = np.degrees(theta3)
theta4_deg = np.degrees(theta4)
theta5_deg = np.degrees(theta5)
theta6_deg = np.degrees(theta6)

print(f"Calculated joint angles (in degrees):")
print(f"Theta 1: {theta1_deg:.2f}")
print(f"Theta 2: {theta2_deg:.2f}")
print(f"Theta 3: {theta3_deg:.2f}")
print(f"Theta 4: {theta4_deg:.2f}")
print(f"Theta 5: {theta5_deg:.2f}")
print(f"Theta 6: {theta6_deg:.2f}")
