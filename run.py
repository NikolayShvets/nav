from typing import Tuple
from scipy.integrate import solve_ivp
from filterpy.kalman import UnscentedKalmanFilter, JulierSigmaPoints
from filterpy.common import Q_discrete_white_noise
import numpy as np
from pyquaternion import Quaternion
from utils import (rotate_matrix_b2g, roll_yaw_pitch,
                   get_ug, ryp2quaternion)

# ------------------------------------------------- #
# ---------------НАЧАЛЬНЫЕ УСЛОВИЯ----------------- #
# ------------------------------------------------- #

# ----------------НАЧАЛЬНЫЕ УГЛЫ------------------- #
init_roll, init_yaw, init_pitch = 0.0, 0.0, 0.0
# ------------------------------------------------- #

# ----------НАЧАЛЬНЫЕ УГЛОВЫЕ СКОРОСТИ------------- #
init_wr, init_wy, init_wp = 0.0, 0.0, 0.0
# ------------------------------------------------- #

# ---КВАТЕРНИОН ОРИЕНТАЦИИ ССК ОТНОСИТЕЛЬНО ГСК---- #
l = ryp2quaternion(init_roll, init_yaw, init_pitch)
# ------------------------------------------------- #

# ----------НАЧАЛЬНЫЙ ВЕКТОР СОСТОЯНИЯ ЛА---------- #
initial_state = np.array([*l, init_wr, init_wy, init_wp], dtype=float)
# ------------------------------------------------- #

# ---------------ВРЕМЯ ИНТЕГРИРОВАНИЯ-------------- #
t_span: Tuple[float, float] = 0.0, 10.0
# ------------------------------------------------- #

# ---ТОЧКИ ПОЛУЧЕНИЯ РЕЗУЛЬТАТОВ ИНТЕГРИРОВАНИЯ---- #
t_eval = np.arange(0, 11, 1)
# ------------------------------------------------- #

u = 7.292115e-5
lat = 0.96
# ------------------------------------------------- #
# ------------------------------------------------- #
# ------------------------------------------------- #


# -------------------ДИНМИКА ЛА-------------------- #
def quadcopter(t: float, x: np.array) -> np.array:
    y = np.zeros(x.shape)
    l = Quaternion(x[0], x[1], x[2], x[3])
    w = Quaternion(0, x[4], x[5], x[6])
    dldt = 0.5 * w * l
    y[0] = dldt[0]
    y[1] = dldt[1]
    y[2] = dldt[2]
    y[3] = dldt[3]
    y[4] = 0.0
    y[5] = 0.0
    y[6] = 0.0
    return y
# ------------------------------------------------- #


# ------------------ДИНМИКА БИНС------------------- #
def bins(t, l, w_bins):
    l = Quaternion(l[0], l[1], l[2], l[3])
    w = Quaternion(0, w_bins[0], w_bins[1], w_bins[2])
    wg_earth = Quaternion(0, u * np.cos(lat), u * np.sin(lat), 0)
    dldt = 0.5 * (l * w - wg_earth * l + l * (1 - l.norm))
    y = np.array([dldt.w, dldt.x, dldt.y, dldt.z], dtype=float)
    return y
# ------------------------------------------------- #


# ------------------------------------------------- #
# -----------------ФИЛЬТР КАЛМАНА------------------ #
# ------------------------------------------------- #


# -----------------ФУНКЦИЯ ПРОЦЕССА---------------- #
def state_transition_function(x: np.array, dt: float, delta_w: np.array,):
    psi = np.zeros(x.shape)
    psi[0] = delta_w[2] * x[1] - delta_w[1] * x[2]
    psi[1] = delta_w[1] * x[2] - delta_w[2] * x[1]
    psi[2] = delta_w[1] * x[0] - delta_w[0] * x[1]
    return psi
# ------------------------------------------------- #


# ----------------ФУНКЦИЯ НАБЛЮДЕНИЯ--------------- #
def measurement_function(x: np.array) -> np.array:
    return x
# ------------------------------------------------- #


# -----------АЛГОРИТМ ВЫБОРА СИГМА-ТОЧЕК----------- #
points = JulierSigmaPoints(3, kappa=0)
# ------------------------------------------------- #


# ----------------------ФИЛЬТР--------------------- #
filter = UnscentedKalmanFilter(dim_x=3,
                               dim_z=3,
                               dt=1.0,
                               hx=measurement_function,
                               fx=state_transition_function,
                               points=points)
filter.Q = Q_discrete_white_noise(dim=2,
                                  dt=1.0,
                                  var=1e-2,
                                  block_size=3)
filter.R = np.array([[1e-2 * 1e-2]])
filter.x = np.array([0.0, 0.0, 0.0])
filter.P = np.array([
    [1e-2, 0.0, 0.0],
    [0.0, 1e-2, 0.0],
    [0.0, 0.0, 1e-2]
])
filtered_state = []
state_covariance_history = []
# ------------------------------------------------- #

quadcopter_solve = solve_ivp(fun=quadcopter,
                             y0=initial_state,
                             t_span=t_span,
                             t_eval=t_eval,
                             atol=1e-12,
                             rtol=1e-12)

for i in range(quadcopter_solve.y.shape[-1]):
    column = quadcopter_solve.y[:, i]
    # вектор углов поворота связанных осей относительно осей сопровождающего трехгранника
    q = roll_yaw_pitch(column[:4])
    print(q)
    # вектор угловых скоростей связанных осей относительно осей сопровождающего трехгранника
    wg = column[4:]
    wg.shape = 3, 1
    # вектор угловой скорости Земли в ССК
    ub = rotate_matrix_b2g(*q, inv=True).dot(get_ug(lat))
    # вектор угловых скоростей в ССК
    wb = rotate_matrix_b2g(*q, inv=True).dot(wg)
    # вектор нормальных ошибок определения угловых скоростей с помощью ДУС
    eps = np.random.normal(0.0, 1e-2, 3)
    eps.shape = 3, 1
    # измерение ДУС
    print("Скороть вращения Земли в ССК:", ub, sep="\n")
    measurement = wb + ub + eps
    print(f"Измерение ДУС на время t{t_eval[i]}:", measurement, sep="\n")

    try:
        bins_solve = solve_ivp(fun=bins,
                               y0=column[:4],
                               t_span=[t_eval[i], t_eval[i+1]],
                               t_eval=[t_eval[i], t_eval[i+1]],
                               args=(measurement,),
                               atol=1e-12,
                               rtol=1e-12)
    except IndexError:
        break
    else:
        r_bins, y_bins, p_bins = roll_yaw_pitch(bins_solve.y[:, -1])
        print(f"Углы по БИНС на время t{t_eval[i + 1]}:",
              f"Крен: {r_bins}", f"Рыскание: {y_bins}", f"Тангаж: {p_bins}", sep="\n\t")

        q_bins = np.array([r_bins, y_bins, p_bins], dtype=float)
        q_magnetometer = wb + np.random.normal(0.0, 1e-3, 1)
        z = q_bins - q_magnetometer
        filter.predict(dt=1.0, delta_w=z)
        filter.update(z)

