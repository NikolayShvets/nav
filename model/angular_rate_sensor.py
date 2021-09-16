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
        dl[0] = -1 / 2 * (wr * l[1] + wy * l[2] + wp * l[3])
        dl[1] = +1 / 2 * (wr * l[0] + wy * l[3] - wp * l[2])
        dl[2] = +1 / 2 * (wy * l[0] + wp * l[1] - wr * l[3])
        dl[3] = +1 / 2 * (wp * l[0] + wr * l[2] - wy * l[1])

        return dl

    def roll_yaw_pitch(self):
        l = self.state
        l_norm = l / sum([li ** 2 for li in l])
        r = np.arcsin(
            (2 * l_norm[0] * l_norm[1] - 2 * l_norm[2] * l_norm[3]) /
            (2 * l_norm[0] ** 2 + 2 * l_norm[2] ** 2 - 1)
        )
        y = np.arcsin(
            (2 * l_norm[0] * l_norm[2] - 2 * l_norm[1] * l_norm[3]) /
            (2 * l_norm[0] ** 2 + 2 * l_norm[1] ** 2 - 1)
        )
        p = np.arcsin(
            (2 * l_norm[1] * l_norm[2] + 2 * l_norm[0] * l_norm[3])
        )
        return r, y, p
