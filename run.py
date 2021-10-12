from typing import Tuple
from scipy.integrate import solve_ivp
from filterpy.kalman import UnscentedKalmanFilter, JulierSigmaPoints, KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
from pyquaternion import Quaternion
from utils import (rotate_matrix_b2g, roll_yaw_pitch,
                   get_ug, ryp2quaternion, length)
import matplotlib.pyplot as plt

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
t_span: Tuple[float, float] = 0.0, 100
# ------------------------------------------------- #

# ---ТОЧКИ ПОЛУЧЕНИЯ РЕЗУЛЬТАТОВ ИНТЕГРИРОВАНИЯ---- #
t_eval = np.arange(0, 101, 1)
# ------------------------------------------------- #

u = 7.292115e-5
lat = 0.96
processNoise = 1e-3


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
def state_transition_function(x: np.array, dt: float, delta_q: np.array, drift: np.array):
    betta = 1e-4
    psi = np.zeros(x.shape)
    psi[0] = delta_q[2] * x[1] - delta_q[1] * x[2] + drift[0]
    psi[1] = delta_q[1] * x[2] - delta_q[2] * x[1] + drift[1]
    psi[2] = delta_q[1] * x[0] - delta_q[0] * x[1] + drift[2]
    psi[3] = betta * x[3] + np.random.normal(0.0, 1e-3, 1)
    return psi


# ------------------------------------------------- #


# ----------------ФУНКЦИЯ НАБЛЮДЕНИЯ--------------- #
def measurement_function(x: np.array) -> np.array:
    return x


# ------------------------------------------------- #


# -----------АЛГОРИТМ ВЫБОРА СИГМА-ТОЧЕК----------- #
points = JulierSigmaPoints(4, kappa=0)
# ------------------------------------------------- #


# ----------------------ФИЛЬТР--------------------- #
filter = UnscentedKalmanFilter(dim_x=4,
                               dim_z=4,
                               dt=1e0,
                               hx=measurement_function,
                               fx=state_transition_function,
                               points=points)

# filter = KalmanFilter(dim_x=4,
#                       dim_z=4)

filter.x = np.array([0, 0, 0, 0])

# filter.H = np.array([
#     [0, 0, 0, 0],
#     [0, 1, 0, 0],
#     [0, 0, 0, 0],
#     [0, 0, 0, 1],
# ])
# filter.P = np.array([
#     [1e-1, 0, 0, 0],
#     [0, 1e-1, 0, 0],
#     [0, 0, 1e-1, 0],
#     [0, 0, 0, 1e-1]
# ])
# filter.R = np.array([
#     [1e-5, 1e-5, 1e-5, 1e-5],
#     [1e-5, 1e-5,  1e-5, 1e-5],
#     [1e-5, 1e-5, 1e-5, 1e-5],
#     [1e-5, 1e-5,  1e-5, 1e-4 * 1e-4]
# ])
filter.Q = Q_discrete_white_noise(dim=4,
                                  dt=1e0,
                                  block_size=1,
                                  var=processNoise)
filtered_state = []
state_covariance_history = []

q_correction = []
bins_q = []
q_real = []

m = []
# ------------------------------------------------- #

quadcopter_solve = solve_ivp(fun=quadcopter,
                             y0=initial_state,
                             t_span=t_span,
                             t_eval=t_eval,
                             atol=1e-6,
                             rtol=1e-6)

for i in range(quadcopter_solve.y.shape[-1]):
    column = quadcopter_solve.y[:, i]
    # вектор углов поворота связанных осей относительно осей сопровождающего трехгранника
    q = roll_yaw_pitch(column[:4])
    q_real.append(q[1])
    # вектор угловых скоростей связанных осей относительно осей сопровождающего трехгранника
    wg = column[4:]
    wg.shape = 3, 1
    # вектор угловой скорости Земли в ССК
    ub = rotate_matrix_b2g(*q, inv=True).dot(get_ug(lat))
    # вектор угловых скоростей в ССК
    wb = rotate_matrix_b2g(*q, inv=True).dot(wg)
    # вектор нормальных ошибок определения угловых скоростей с помощью ДУС
    eps = np.random.normal(0.0, 1e-5, 3)
    eps.shape = 3, 1
    drift = np.random.normal(0.0, 1e-3, 3)
    drift.shape = 3, 1
    # измерение ДУС
    print("Скороть вращения Земли в ССК:", ub, sep="\n")
    measurement = wb + ub + drift + eps
    print(f"Измерение ДУС на время t{t_eval[i]}:", measurement, sep="\n")

    try:
        bins_solve = solve_ivp(fun=bins,
                               y0=column[:4],
                               t_span=[t_eval[i], t_eval[i + 1]],
                               t_eval=[t_eval[i], t_eval[i + 1]],
                               args=(measurement,),
                               atol=1e-6,
                               rtol=1e-6)
    except IndexError:
        break

    r_bins, y_bins, p_bins = roll_yaw_pitch(bins_solve.y[:, -1])
    bins_q.append(y_bins)
    print(f"Углы по БИНС на время t{t_eval[i + 1]}:",
          f"Крен: {r_bins}", f"Рыскание: {y_bins}", f"Тангаж: {p_bins}", sep="\n\t")

    q_bins = np.array([r_bins, y_bins, p_bins], dtype=float)
    q_magnetometer = q + np.random.normal(0.0, 2.5e-3, 3)
    delta_q = q_bins - q_magnetometer
    m.append(filter.mahalanobis)

    # filter.F = np.array([
    #     [0, delta_q[2], -delta_q[1], 0],
    #     [0, delta_q[0], -delta_q[2], 0],
    #     [0, delta_q[1], -delta_q[0], 0],
    #     [0, 0, 0, 1e-3]
    # ])

    # z = np.array([
    #     [delta_q[0]],
    #     [delta_q[1]],
    #     [delta_q[2]],
    #     [*np.random.normal(0.0, 1e-4, 1)]
    # ])
    # filter.predict()
    # filter.update(z)

    delta_q = np.array([delta_q[0], delta_q[1], delta_q[2], 0])
    filter.predict(delta_q=delta_q, drift=drift)
    filter.update(delta_q)

    filtered_state.append(filter.x)
    state_covariance_history.append(filter.P)
    q_correction.append((q_bins[1] - filter.x[1]))

plt.plot(q_real[1:], label="Истинное угловое положение", color="#99AAFF")
plt.plot(q_correction, label="Скорректированное угловое положение", color="#FF6633")
plt.plot(bins_q, label="Угловое положение по БИНС", color="#224411")
# plt.plot(m, label="Коррекции", color="#215681")
plt.legend()
plt.show()
