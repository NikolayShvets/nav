import numpy as np


def rotate_matrix_b2g(r, y, p, inv: bool = False):
    c = np.cos
    s = np.sin
    m = np.array([
        [c(y) * c(p), -c(r) * c(y) * s(p) + s(r) * s(y), s(r) * c(y) * s(p) + c(r) * s(y)],
        [s(p), c(r) * c(p), -s(r) * c(p)],
        [-c(p) * s(y), c(r) * s(y) * s(p) + s(r) * c(y), -s(r) * s(y) * s(p) + c(r) * c(y)]
    ])
    if inv:
        return np.linalg.inv(m)
    return m
