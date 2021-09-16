# import numpy as np
# from pyquaternion import Quaternion
#
#
# def get_q(x: np.array) -> Quaternion:
#     h_r = x[0] / 2
#     h_y = x[1] / 2
#     h_p = x[2] / 2
#     l0 = (np.cos(h_r) * np.cos(h_y) * np.cos(h_p) -
#           np.sin(h_r) * np.sin(h_y) * np.sin(h_p))
#     l1 = (np.sin(h_r) * np.cos(h_y) * np.cos(h_p) +
#           np.cos(h_r) * np.sin(h_y) * np.sin(h_p))
#     l2 = (np.cos(h_r) * np.sin(h_y) * np.cos(h_p) +
#           np.sin(h_r) * np.cos(h_y) * np.sin(h_p))
#     l3 = (np.cos(h_r) * np.cos(h_y) * np.sin(h_p) -
#           np.sin(h_r) * np.sin(h_y) * np.cos(h_p))
#
#     q = Quaternion([l0, l1, l2, l3])
#
#     return q
