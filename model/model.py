from typing import Tuple
import numpy as np
from scipy.integrate import solve_ivp


class Model:
    def __init__(self,
                 state: np.array):
        self.init_state = state
        self.results = []

    def increment(self, *args, **kwargs) -> np.array:
        pass

    def compute(self, t_span: Tuple[float, float], t_eval=None):
        solve = solve_ivp(fun=self.increment,
                          y0=self.init_state,
                          t_span=t_span,
                          t_eval=t_eval,
                          atol=1e-12,
                          rtol=1e-12)
        return solve


