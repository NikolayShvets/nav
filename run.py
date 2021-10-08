from scipy.integrate import solve_ivp

from model import Quadcopter
import numpy as np
import quaternion
from pyquaternion import Quaternion
from utils import rotate_matrix_b2g, roll_yaw_pitch, length, get_ug

x = np.array([
    1.0,  # r
    1.0,  # y
    -1.0,  # p
    1.0,  # wr
    2.0,  # wy
    1.0  # wp
], dtype=float)
quadcopter = Quadcopter(state=x)

solve = quadcopter.compute(t_span=(0.0, 10.0),
                           t_eval=np.arange(0.0, 11, 1))

for i in range(solve.y.shape[-1] + 1):
    column = solve.y[:, i]
    # вектор углов поворота связанных осей относительно осей сопровождающего трехгранника
    q = roll_yaw_pitch(column[:4])
    print(q)
    # вектор угловых скоростей связанных осей относительно осей сопровождающего трехгранника
    wg = column[4:]
    wg.shape = 3, 1
    # вектор угловой скорости Земли в ССК
    ub = rotate_matrix_b2g(*q, inv=True).dot(get_ug(quadcopter.lat))
    # вектор угловых скоростей в ССК
    wb = rotate_matrix_b2g(*q, inv=True).dot(wg)
    # вектор нормальных ошибок определения угловых скоростей с помощью ДУС
    eps = np.random.normal(0.0, 1e-2, 3)
    eps.shape = 3, 1
    # измерение ДУС
    measurement = wb #+ ub + eps
    print(measurement)

    init_l = column[:4]

    def bins(t, l, w):
        l = Quaternion(l[0], l[1], l[2], l[3])
        u = 7.292115e-5
        lat = quadcopter.lat
        w = Quaternion(0, w[0], w[1], w[2])
        delta_w = Quaternion(0, u * np.cos(lat), u * np.sin(lat), 0)
        dldt = 0.5 * (l * w - delta_w * l + l * (1 - l.norm))
        y = np.array([dldt.w, dldt.x, dldt.y, dldt.z], dtype=float)
        return y

    l_solve = solve_ivp(fun=bins,
                        y0=init_l,
                        t_span=(solve.t[i], solve.t[i+1]),
                        t_eval=[solve.t[i], solve.t[i+1]],
                        atol=1e-12,
                        rtol=1e-12,
                        args=(measurement,))
    print(roll_yaw_pitch(l_solve.y[:, -1]))
    # print(l_solve.y)
    print(solve.t[i], "-->", solve.t[i+1])

