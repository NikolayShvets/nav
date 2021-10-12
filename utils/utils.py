from typing import Tuple

import numpy as np


def ryp2quaternion(r: float, y: float, p: float) -> np.array:
    l0 = (np.cos(r / 2) * np.cos(y / 2) * np.cos(p / 2) -
          np.sin(r / 2) * np.sin(y / 2) * np.sin(p / 2))
    l1 = (np.sin(r / 2) * np.cos(y / 2) * np.cos(p / 2) +
          np.cos(r / 2) * np.sin(y / 2) * np.sin(p / 2))
    l2 = (np.cos(r / 2) * np.sin(y / 2) * np.cos(p / 2) +
          np.sin(r / 2) * np.cos(y / 2) * np.sin(p / 2))
    l3 = (np.cos(r / 2) * np.cos(y / 2) * np.sin(p / 2) -
          np.sin(r / 2) * np.sin(y / 2) * np.cos(p / 2))

    return np.array([l0, l1, l2, l3], dtype=float)


def rotate_matrix_b2g(r, y, p, inv: bool = False) -> np.array:
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


def roll_yaw_pitch(l: np.array) -> Tuple[float, float, float]:
    l_norm = l / sum([li ** 2 for li in l])
    r = np.arctan2(
        (2 * l_norm[0] * l_norm[1] - 2 * l_norm[2] * l_norm[3]),
        (2 * l_norm[0] ** 2 + 2 * l_norm[2] ** 2 - 1)
    )
    y = np.arctan2(
        (2 * l_norm[0] * l_norm[2] - 2 * l_norm[1] * l_norm[3]),
        (2 * l_norm[0] ** 2 + 2 * l_norm[1] ** 2 - 1)
    )
    p = np.arcsin(
        (2 * l_norm[1] * l_norm[2] + 2 * l_norm[0] * l_norm[3])
    )
    # r = (r + np.pi * 2) % (np.pi * 2)
    # y = (y + np.pi * 2) % (np.pi * 2)
    # p = (p + np.pi * 2) % (np.pi * 2)
    return r, y, p


def get_ug(lat: float) -> np.array:
    # угловая скорость вращения Земли
    u = 7.292115e-5
    ug = np.array([
        [u * np.cos(lat)],
        [u * np.sin(lat)],
        [0.0]
    ], dtype=float)
    return ug


def length(x: np.array) -> float:
    return sum([item**2 for item in x])**0.5
