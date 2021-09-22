import numpy as np


class WhiteNoise:
    __mean = 0.0

    def __init__(self, std: float, size: int = 1):
        self.__std = std
        self.__size = size

    def __call__(self, std: float):
        return np.random.normal(self.__mean, self.__std, self.__size)

