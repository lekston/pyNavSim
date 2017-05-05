from abc import abstractmethod
import numpy as np


class Regulator(object):

    @abstractmethod
    def update(self, system, demand):
        return system


class RollRegulator(Regulator):

    def __init__(self, phi_max=40, p_max=40, p_dot_max=30):
        self._phi_max    = np.deg2rad(phi_max)      # 40 deg by default
        self._p_max      = np.deg2rad(p_max)        # 40 deg/sec by default
        self._p_dot_max  = np.deg2rad(p_dot_max)    # 30 deg/sec^2 by default

    @property
    def phi_max(self):
        return self._phi_max

    @property
    def p_max(self):
        return self._p_max

    @property
    def p_dot_max(self):
        return self._p_dot_max

    def update(self, system, roll_dem):

        roll = system.state_dict['phi']
        roll_rate_cmd = roll_dem - roll  # direct gain (k_p = 1)
        return roll_rate_cmd


class L1NavRegulator(Regulator):

    def __init__(self):
        self.L1_period = 20     # sec
        self.L1_damping = 0.75  # 'dimless'

    def update(self, system, cur_leg):

        roll_dem = 0
        return roll_dem


rollReg = RollRegulator()
navReg = L1NavRegulator()

