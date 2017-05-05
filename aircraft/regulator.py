from abc import abstractmethod
import numpy as np


class Regulator(object):

    @abstractmethod
    def __init__(self):
        self._par_list = []
        self._log_dict = {}

    @abstractmethod
    def update(self, system, demand):
        return

    @property
    def par_list(self):
        return self._par_list

    @property
    def current_val_dict(self):
        return self._log_dict


class RollRegulator(Regulator):

    def __init__(self, phi_max=40, p_max=40, p_dot_max=30):
        super(RollRegulator, self).__init__()
        self._phi_max    = np.deg2rad(phi_max)      # 40 deg by default
        self._p_max      = np.deg2rad(p_max)        # 40 deg/sec by default
        self._p_dot_max  = np.deg2rad(p_dot_max)    # 30 deg/sec^2 by default
        self._par_list   = ['p_dot_dem']
        self._log_dict   = {'p_dot_dem': 0}

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
        roll_rate_cmd = roll_dem - roll  # direct gain (k_p = 1) TODO NEED DAMPING!
        self._log_dict['p_dot_dem'] = roll_rate_cmd
        return roll_rate_cmd


class L1NavRegulator(Regulator):

    def __init__(self):
        super(L1NavRegulator, self).__init__()
        self.L1_period      = 20     # sec
        self.L1_damping     = 0.75  # 'dimless'
        self._par_list      = ['L1_out1', 'L1_out2']
        self._log_dict      = {'L1_out1':0, 'L1_out2':0}

    def update(self, system, cur_leg):

        roll_dem = 0
        self._log_dict['L1_out1'] = roll_dem - 3
        self._log_dict['L1_out2'] = roll_dem + 3
        return roll_dem


rollReg = RollRegulator()
navReg = L1NavRegulator()

