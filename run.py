from model import Quadcopter
import numpy as np
from utils import rotate_matrix_b2g, roll_yaw_pitch, length


x = np.array([
    0.0,  # r
    0.0,  # y
    0.0,  # p
    0.0,  # wr
    0.0,  # wy
    0.0   # wp
], dtype=float)
quadcopter = Quadcopter(state=x)

evaluation, t_eval = quadcopter.compute(t_span=(0.0, 11.5),
                                        t_eval=np.arange(0.0, 1.5, 0.12))

print(evaluation, t_eval)
# while rk.t < rk.t_bound:
#     rk.step()
#     # вектор углов поворота связанных осей относительно осей сопровождающего трехгранника
#     q = roll_yaw_pitch(rk.y[:4])
#     # вектор угловых скоростей связанных осей относительно осей сопровождающего трехгранника
#     wg = rk.y[4:8]
#     wg.shape = 3, 1
#     # вектор угловой скорости Земли в ССК
#     ub = rotate_matrix_b2g(*q, inv=True).dot(quadcopter.get_ug())
#     # вектор угловых скоростей в ССК
#     wb = rotate_matrix_b2g(*q, inv=True).dot(wg)
#     # вектор нормальных ошибок определения угловых скоростей с помощью ДУС
#     eps = np.random.normal(0.0, 1e-2, 3)
#     eps.shape = 3, 1
#
#     measurement = wb + ub + eps
#     print(rk.t)
