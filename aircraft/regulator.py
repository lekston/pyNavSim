from abc import abstractmethod
import numpy as np
import utils.basic_nav as bn

class Regulator(object):

    @abstractmethod
    def __init__(self):
        self._par_list = []
        self._log_dict = {}

    @abstractmethod
    def update(self, system, demand, observables):
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
        self._prev_err   = 0
        self._prev_state = 0
        self._k_p        = 0.5
        self._k_d        = -100     # TODO compensate the time scale
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

    def update(self, system, demand, observables):

        # TODO consider time scale of the regulator

        roll = system.state_dict['phi']
        roll_dem = demand  # TODO rate-limit the roll demand
        err = roll_dem - roll
        err_dot_raw = self._prev_err - err
        err_dot_flt = 0.8 * self._prev_state + 0.2 * err_dot_raw

        roll_rate_cmd = self._k_p * err + self._k_d * err_dot_flt
        self._log_dict['p_dot_dem'] = roll_rate_cmd

        self._prev_state = err_dot_flt
        self._prev_err   = err
        return roll_rate_cmd


class L1NavRegulator(Regulator):

    def __init__(self):
        super(L1NavRegulator, self).__init__()
        self.L1_period      = 20     # sec
        self.L1_damping     = 0.75   # dimensionless
        self._par_list      = ['L1_out1', 'L1_out2']
        self._log_dict      = {'L1_out1': 0, 'L1_out2': 0}

    @staticmethod
    def nav_roll_dem(latAccDem):
        res = np.arctan(latAccDem / 9.81)  # [rad]
        res = np.clip(res, -np.pi / 2, np.pi / 2)
        return res

    def update(self, system, demand, observables):

        cur_leg = demand

        self.update_wp_nav(system, cur_leg[0], cur_leg[1], observables)

        if 1:
            tmp = 0
            self._log_dict['L1_out1'] = tmp - 3
            self._log_dict['L1_out2'] = tmp + 3

        latAccDem = 0

        return self.nav_roll_dem(latAccDem)

    def update_wp_nav(self, system, prev_wp, next_wp, observables):

        psi = system.state_dict['psi']
        tas = observables['TAS']
        wind = observables['wind']
        gnd_spd_vect = observables['gnd_spd']

        if bn.norm_2d(gnd_spd_vect) < 0.1:
            raise RuntimeError("Unable to fly forward due to wind")
            # APM assumes minimal gnd_spd along heading to keep on computing

        wind_vec = np.array([np.sin(wind[0]), np.cos(wind[0])]) * wind[1]
        tas_vec = np.array([np.sin(psi), np.cos(psi)]) * tas
        current_loc_vec = np.array([system.state_dict['x'],
                                     system.state_dict['y']])

        K_L1 = 4.0 * self.L1_damping**2

        target_bearing = bn.get_bearing(prev_wp, next_wp)
        or2target_vec = next_wp - prev_wp                  # AB
        or2target_dist = bn.norm_2d(or2target_vec)
        or2target_vec *= 1./or2target_dist                 # normalize

        origin_vec = current_loc_vec - prev_wp             # A_air

        # TODO: potential APM bug! (missing 0.5 factor)
        crosstrack_err = np.cross(origin_vec, or2target_vec) * 0.5   # (A_air % AB) (APM is missing the 0.5 factor)

        return 0


rollReg = RollRegulator()
navReg = L1NavRegulator()

