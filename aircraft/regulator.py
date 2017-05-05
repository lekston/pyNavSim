from abc import abstractmethod
import numpy as np


class Regulator(object):

    @abstractmethod
    def update(self, system, demand):
        return system


class RollRegulator(Regulator):

    def __init__(self):
        self.phi_max    = np.deg2rad(40)    # 40 deg
        self.p_max      = np.deg2rad(40)    # 40 deg/sec
        self.p_dot_max  = np.deg2rad(30)    # 30 deg/sec^2

    @property
    def p1(self):
        return self.phi_max

    @property
    def p2(self):
        return self.p_max

    @property
    def p3(self):
        return self.p_dot_max

    def update(self, system, demand):

        return system


class L1NavRegulator(Regulator):

    def __init__(self):
        self.L1_period = 20     # sec
        self.L1_damping = 0.75  #'dimless'

    def update(self, system, demand):

        return system


rollReg = RollRegulator()
navReg = L1NavRegulator()

