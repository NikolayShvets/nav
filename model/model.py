import numpy as np


class Model:
    def __init__(self,
                 state: np.array,
                 t_start: float = 0.0,
                 t_finish: float = 10.0,
                 t_step: float = 1.0):
        self.t_start = t_start
        self.t_finish = t_finish
        self.t_step = t_step
        self.init_state = state
        self.results = []

    def increment(self, *args, **kwargs) -> np.array:
        pass

    @property
    def order(self):
        return self.state.shape[0]

