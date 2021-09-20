import numpy as np
from .model import Model


class AngularRateSensor(Model):
    def __init__(self, state: np.array,
                 t_start: float = 0.0,
                 t_finish: float = 10.0,
                 t_step: float = 1.0):
        r, y, p = state
        l0 = (np.cos(r / 2) * np.cos(y / 2) * np.cos(p / 2) -
              np.sin(r / 2) * np.sin(y / 2) * np.sin(p / 2))
        l1 = (np.sin(r / 2) * np.cos(y / 2) * np.cos(p / 2) +
              np.cos(r / 2) * np.sin(y / 2) * np.sin(p / 2))
        l2 = (np.cos(r / 2) * np.sin(y / 2) * np.cos(p / 2) +
              np.sin(r / 2) * np.cos(y / 2) * np.sin(p / 2))
        l3 = (np.cos(r / 2) * np.cos(y / 2) * np.sin(p / 2) -
              np.sin(r / 2) * np.sin(y / 2) * np.cos(p / 2))
        l = [l0, l1, l2, l3]
        super().__init__(np.array(l),
                         t_start, t_finish, t_step)

    def increment(self, t, l: np.array, w) -> np.array:
        dl = np.zeros(l.shape)
        wr, wy, wp = w
        dl[0] = -0.5 * (wr * l[1] + wy * l[2] + wp * l[3])
        dl[1] = +0.5 * (wr * l[0] + wy * l[3] - wp * l[2])
        dl[2] = +0.5 * (wy * l[0] + wp * l[1] - wr * l[3])
        dl[3] = +0.5 * (wp * l[0] + wr * l[2] - wy * l[1])

        return dl

    @classmethod
    def roll_yaw_pitch(cls, l):
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
        # if r < 0.0:
        #     r += -2 * np.pi
        return r, y, p
