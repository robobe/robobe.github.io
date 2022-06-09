import cv2
import numpy as np

R = np.array([[-0.0813445156856268], [-2.5478950926311636], [1.7376856594745234]], dtype=np.float32)
T = np.array([[10.838262901867047], [-6.506593974297687], [60.308121310607724]], dtype=np.float32)

world_point = [13, 0, 0]

rvec_matrix = cv2.Rodrigues(R)[0]
rmat = np.matrix(rvec_matrix)
tmat = np.matrix(T)
pmat = np.matrix(np.array([[world_point[0]], [world_point[1]], [world_point[2]]], dtype=np.float32))

# world coordinate to camera coordinate
cam_point = rmat * pmat + tmat
print(cam_point)

# camera coordinate to world coordinate
world_point = rmat ** -1 * (cam_point - tmat)