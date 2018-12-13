import numpy as np

def getTransformationMatrix(theta, x, y):
    radius = theta / 180 * np.pi
    R = np.mat([[np.cos(radius), -np.sin(radius), 0, 0], [np.sin(radius), np.cos(radius), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = np.eye(4, 4, dtype=float)
    T[0,3] = x
    T[1,3] = y
    H = T * R
    return H

def getGoalOrientation(goal_x, goal_y, x, y):
    return np.arctan2(goal_y - y, goal_x - x) / np.pi * 180

def getRotationAngle(H):
    s = H[1,0]
    c = H[0,0]
    return np.arctan2(s, c) / np.pi * 180

'''
angle = getGoalOrientation(30, 30, 10, 20)
wHr = getTransformationMatrix(0, 10, 20)
wHg = getTransformationMatrix(angle, 30, 30)

gHr = wHg.I * wHr
rHg = gHr.I

print(getRotationAngle(rHg))
'''

