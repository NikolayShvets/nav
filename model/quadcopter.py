from .model import Model
import numpy as np
import quaternion
from pyquaternion import Quaternion


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

    def __init__(self, state: np.array):
        r, y, p = state[:3]
        self.l = quaternion.from_euler_angles(r, y, p)

        r, y, p = state[:3]
        l0 = (np.cos(r / 2) * np.cos(y / 2) * np.cos(p / 2) -
              np.sin(r / 2) * np.sin(y / 2) * np.sin(p / 2))
        l1 = (np.sin(r / 2) * np.cos(y / 2) * np.cos(p / 2) +
              np.cos(r / 2) * np.sin(y / 2) * np.sin(p / 2))
        l2 = (np.cos(r / 2) * np.sin(y / 2) * np.cos(p / 2) +
              np.sin(r / 2) * np.cos(y / 2) * np.sin(p / 2))
        l3 = (np.cos(r / 2) * np.cos(y / 2) * np.sin(p / 2) -
              np.sin(r / 2) * np.sin(y / 2) * np.cos(p / 2))

        state = np.concatenate((np.array([l0, l1, l2 ,l3], dtype=float), state[3:]))
        super().__init__(state)
        # self.l = Quaternion(l0, l1, l2, l3)
        # self.t_prev = None

    def increment(self, t: float, x: np.array) -> np.array:
        # if self.t_prev is None:
        #     self.t_prev = t
        # self.l.integrate(x, t-self.t_prev)
        # self.t_prev = t
        #
        # yaw, pitch, roll = self.l.yaw_pitch_roll
        # roll = (roll + np.pi * 2) % (np.pi * 2)
        # yaw = (yaw + np.pi * 2) % (np.pi * 2)

        y = np.zeros(x.shape)

        l = Quaternion(x[0], x[1], x[2], x[3])
        w = Quaternion(0, x[4], x[5], x[6])
        dldt = 0.5 * l * w
        y[0] = dldt[0]
        y[1] = dldt[1]
        y[2] = dldt[2]
        y[3] = dldt[3]
        y[4] = 0.0
        y[5] = 0.0
        y[6] = 0.0
        # y[4] = (1 / self.jx *
        #         (self.r * 0.0 + (self.jr + self.jp) *
        #          x[5] * (self.W1 - self.W2 + self.W3 - self.W4) +
        #          x[4] * x[5] * (self.jz - self.jy)))
        #
        # y[5] = 1 / self.jy * (self.r * 0.0 + x[4] * x[3] * (self.jy - self.jx))
        #
        # y[6] = (1 / self.jz *
        #         (self.r * 0.0 + (self.jr + self.jp) *
        #          x[3] * (-self.W1 + self.W2 - self.W3 + self.W4) +
        #          x[3] * x[5] * (self.jx - self.jz)))
        return y
