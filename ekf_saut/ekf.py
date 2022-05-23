import numpy as np
from abc import ABC, abstractmethod


class EKF(ABC):
    def __init__(self, u, v ,r, psi):
        self.u = u
        self.v = v
        self.r = r
        self.psi = psi
    
    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def predict(self):
        pass

class EKF_simple(EKF):

    def update()
        x_dot = np.cos(self.psi) - sin(self.psi)
        y_dot = np.sin(self.psi) + np.cos(self.psi)
        psi_dot = r