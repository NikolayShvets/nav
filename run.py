from model import Quadcopter
import numpy as np
from scipy.integrate import RK45
from utils import rotate_matrix_b2g

#              r    y    p    wr   wy   wp
x = np.array([0.0, np.pi/3, 0.0, 1.0, 0.0, 0.0], dtype=float)

quadcopter = Quadcopter(state=x, t_start=0.0, t_step=1, t_finish=10)
rk = RK45(fun=quadcopter.increment,
          t0=quadcopter.t_start,
          y0=quadcopter.init_state,
          t_bound=quadcopter.t_finish,
          first_step=quadcopter.t_step,
          max_step=1,
          vectorized=False)

res = []
while rk.t < rk.t_bound:
    rk.step()
    # вектор углов поворота связанных осей относительно осей сопровождающего трехгранника
    q = quadcopter.ars.roll_yaw_pitch(rk.y[6:10])
    # вектор угловых скоростей связанных осей относительно осей сопровождающего трехгранника
    wg = rk.y[3:6]
    wg.shape = 3, 1
    # вектор угловой скорости Земли в ССК
    ub = rotate_matrix_b2g(*q, inv=True).dot(quadcopter.get_ug())
    # вектор угловых скоростей в ССК
    wb = rotate_matrix_b2g(*q, inv=True).dot(wg)
    # вектор нормальных ошибок определения угловых скоростей с помощью ДУС
    eps = np.random.normal(0.0, 1e-2, 3)
    eps.shape = 3, 1

    measurement = wb + ub + eps
    quadcopter.ars.measurements.append(measurement)
    print(wb)

