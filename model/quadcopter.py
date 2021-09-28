from .model import Model
from .angular_rate_sensor import AngularRateSensor
from utils import rotate_matrix_b2g
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
    # угловая скорость вращения Земли
    U = 7.292115e-5
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

        y[3] = (1 / self.jx *
                (self.r * 0.0 + (self.jr + self.jp) *
                 x[5] * (self.W1 - self.W2 + self.W3 - self.W4) +
                 x[4] * x[5] * (self.jz - self.jy)))

        y[4] = 1 / self.jy * (self.r * 0.0 + x[4] * x[3] * (self.jy - self.jx))

        y[5] = (1 / self.jz *
                (self.r * 0.0 + (self.jr + self.jp) *
                 x[3] * (-self.W1 + self.W2 - self.W3 + self.W4) +
                 x[3] * x[5] * (self.jx - self.jz)))

        ars_y = self.ars.increment(t, x[6:], w=x[3:6])
        y[6:] = ars_y
        return y

    def get_ug(self):
        ug = np.array([
            [self.U * np.cos(self.lat)],
            [self.U * np.sin(self.lat)],
            [0.0]
        ], dtype=float)
        return ug
