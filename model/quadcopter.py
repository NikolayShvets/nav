from .model import Model
from .angular_rate_sensor import AngularRateSensor
import numpy as np


class Quadcopter(Model):
    # широта
    lat = 0.96
    # долгота
    lon = 0.65
    # высота над уровнем поверхности Земли
    h = 150.0
    # радиус Земли
    R = 6371e3
    # ускорение свободного падения
    g = 9.8
    # масса двигаетля
    me = 0.05
    # масса пропеллера
    mp = 0.025
    # радиус корпуса
    rc = 0.15
    # радиус ротора
    rr = 0.05
    # радиус пропеллера
    rp = 0.1
    # длина плеча
    r = 1.0

    # момент интерции по осям x и y
    jx = jy = 1.26e-2
    # момент инерции по оси z
    jz = 2.16e-2
    # момент инерции ротора двигателя
    jr = 6.25e-5
    # момент инерции пропеллера
    jp = 4e-4
    # скорости пропеллеров при зависании
    W1 = W2 = W3 = W4 = 340

    def __init__(self, state: np.array,
                 t_start: float = 0.0,
                 t_finish: float = 50.0,
                 t_step: float = 1.0):
        super().__init__(state,
                         t_start, t_finish, t_step)
        self.ars = AngularRateSensor(state[:3], t_start, t_finish, t_step)
        self.init_state = np.concatenate((state, self.ars.init_state))

    def increment(self, t, x: np.array) -> np.array:
        y = np.zeros(x.shape)
        y[0] = x[3]
        y[1] = x[4]
        y[2] = x[5]
        y[3] = 0.0
        y[4] = 0.0
        y[5] = 0.0

        # y[3] = (1 / self.jx *
        #         (self.r * 0.01 + (self.jr + self.jp) *
        #          x[5] * (self.W1 - self.W2 + self.W3 - self.W4) +
        #          x[4] * x[5] * (self.jz - self.jy)))

        # y[4] = 1 / self.jy * (self.r * 0.01 + x[4] * x[3] * (self.jy - self.jx))
        #
        # y[5] = (1 / self.jz *
        #         (self.r * 0.01 + (self.jr + self.jp) *
        #          x[3] * (-self.W1 + self.W2 - self.W3 + self.W4) +
        #          x[3] * x[5] * (self.jx - self.jz)))

        # y[3] = 0.0 # np.sin(t)
        # y[4] = 0.0 # np.sin(t)
        # y[5] = 0.0 # np.sin(t)

        ars_y = self.ars.increment(t, x[6:], w=x[3:6])
        y[6:] = ars_y
        return y

# class Quadcopter(Model):
#     def __init__(self,
#                  lat: float,
#                  lon: float,
#                  h: float,
#                  initial_state: np.array,
#                  t_start: float = 0.0,
#                  t_finish: float = 100.0,
#                  t_step: float = 1.0):
#         super().__init__(initial_state, t_start, t_finish, t_step)
#
#         self.lat = lat
#         self.lon = lon
#         # Re + h
#         self.h = h
#
#     #  0  1  2  3   4   5    6  7   8    9
#     # [y, p, r, wy, wp, wr, l0, l1, l2, l3]
#     def increment(self, t: float, x: np.array) -> np.array:
#         y = np.zeros(x.shape)
#
#         y[0] = x[3]
#         y[1] = x[4]
#         y[2] = x[5]
#
#         y[3] = 0.0# np.sin(t)
#         y[4] = 0.0# np.cos(t)
#         y[5] = 0.0# np.sin(t) + 0.3 * np.sin(t)
#
#         q = Quaternion(x[6:])
#         w = Quaternion(0, *x[3:6])
#
#         dqdt = 0.5 * (q * w)
#         y[6] = dqdt[0]
#         y[7] = dqdt[1]
#         y[8] = dqdt[2]
#         y[9] = dqdt[3]
#
#         return y
