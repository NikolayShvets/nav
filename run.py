from pyquaternion import Quaternion

from model import Quadcopter
import numpy as np
from scipy.integrate import RK45

U = 7.292115e-5
#              r    y    p    wr   wy   wp
x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)


quadcopter = Quadcopter(state=x, t_start=0.0, t_step=1, t_finish=10)
rk = RK45(fun=quadcopter.increment,
          t0=quadcopter.t_start,
          y0=quadcopter.state,
          t_bound=quadcopter.t_finish,
          first_step=quadcopter.t_step,
          vectorized=False)

while rk.t < rk.t_bound:
    rk.step()
    res = [*rk.y, rk.t]
    quadcopter.results.append(res)
    with open("../../Desktop/res.txt", "a") as f:
        print(*res, sep="|", file=f)
    # quadcopter.state = rk.y[:6]
    quadcopter.ars.state = rk.y[6:]
    # print(quadcopter.ars.roll_yaw_pitch())
    # quadcopter.results.append([*quadcopter.state, rk.t])
    # quadcopter.ars.results.append([*quadcopter.ars.state, rk.t])
print(*quadcopter.results, sep="\n")
print(quadcopter.ars.roll_yaw_pitch())
