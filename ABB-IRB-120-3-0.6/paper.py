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
Pwx, Pwy, Pwz = Pw[0], Pw[1], Pw[2]

theta1 = np.arctan2(Py, Px)

r = np.hypot(Pwx, Pwy)
R = np.sqrt(r**2 + (Pwz - d1)**2)
sin_alpha = (Pwz - d1) / R
cos_alpha = r / R
cos_beta = (a2**2 + R**2 - a3**2 - d4**2)
sin_beta = np.sqrt(1 - cos_beta**2)
alpha = np.arctan2(sin_alpha, cos_alpha)
beta = np.arctan2(sin_beta, cos_beta)

sin_theta2 = (-sin_alpha * cos_beta) + (cos_alpha * sin_beta) 
cos_theta2 = (cos_alpha * cos_beta) + (sin_alpha * sin_beta) 
theta2 = np.arctan2(sin_theta2, cos_theta2) 

print(np.rad2deg(theta1))
print(np.rad2deg(theta2))