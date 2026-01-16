import numpy as np 
# X, Y, Z

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
print(R)