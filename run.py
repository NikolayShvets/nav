from model import Quadcopter
import numpy as np
from scipy.integrate import RK45

U = 7.292115e-5
#              r    y    p    wr   wy   wp
x = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], dtype=float)


quadcopter = Quadcopter(state=x, t_start=0.0, t_step=0.01, t_finish=4.71)
rk = RK45(fun=quadcopter.increment,
          t0=quadcopter.t_start,
          y0=quadcopter.init_state,
          t_bound=quadcopter.t_finish,
          first_step=quadcopter.t_step,
          vectorized=False)

res = []
while rk.t < rk.t_bound:
    rk.step()
    res = [*rk.y, rk.t]
    print(res, quadcopter.ars.roll_yaw_pitch(res[6:10]))
