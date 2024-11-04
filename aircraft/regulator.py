from abc import abstractmethod
import numpy as np
import utils.basic_nav as bn


class Regulator(object):

    @abstractmethod
    def __init__(self):
        self._par_list = []
        self._log_dict = {}

    @abstractmethod
    def update(self, system, dt, demand, observables):
        return

    @property
    def par_list(self):
        return self._par_list

    @property
    def current_val_dict(self):
        return self._log_dict


class RollRegulator(Regulator):

    def __init__(self, phi_max=35, p_max=60, p_dot_max=80):
        super(RollRegulator, self).__init__()
        self._filt_init         = False
        self._prev_phi_dem      = 0
        self._prev_p_dot_cmd    = 0
        self._phi_dem_dot_max   = np.deg2rad(40)    # 40 deg/sec by default
        self._phi_max    = np.deg2rad(phi_max)      # 35 deg by default
        self._p_max      = np.deg2rad(p_max)        # 60 deg/sec by default
        self._p_dot_max  = np.deg2rad(p_dot_max)    # 80 deg/sec^2 by default
        self._prev_err   = 0
        self._prev_state = 0
        self._k_p        = 0.1
        self._k_d        = 3
        self._log_dict   = {'R_p_dot_cmd': 0, 'R_phi_dem_ll': 0}
        self._par_list   = [key for key in self._log_dict.keys()]

    @property
    def phi_max(self):
        return self._phi_max

    @property
    def p_max(self):
        return self._p_max

    @property
    def p_dot_max(self):
        return self._p_dot_max

    def update(self, system, dt, demand, observables):

        roll = system.state_dict['S_phi']

        if not self._filt_init:
            self._filt_init = True
            self._prev_phi_dem = roll   # forces initial error to be 0

        roll_dem = np.clip(demand, -self._phi_max, self._phi_max)
        # rate-limit roll demand
        roll_dem = np.clip(roll_dem, self._prev_phi_dem - self._phi_dem_dot_max * dt,
                                     self._prev_phi_dem + self._phi_dem_dot_max * dt)
        err = roll_dem - roll
        err_dot_raw = (self._prev_err - err)/dt
        err_dot_flt = 0.8 * self._prev_state + 0.2 * err_dot_raw

        p_dot_cmd = self._k_p * err - self._k_d * err_dot_flt

        # low-pass the output to better reflect speed-control characteristics of ailerons
        p_dot_cmd = 0.9 * self._prev_p_dot_cmd + 0.1 * p_dot_cmd

        self._log_dict['R_p_dot_cmd'] = p_dot_cmd
        self._log_dict['R_phi_dem_ll'] = roll_dem

        self._prev_p_dot_cmd = p_dot_cmd
        self._prev_phi_dem = roll_dem
        self._prev_state = err_dot_flt
        self._prev_err   = err

        return p_dot_cmd


# noinspection PyPep8Naming
class L1NavRegulator(Regulator):

    def __init__(self, period=30, damping=0.85):
        super(L1NavRegulator, self).__init__()
        self.L1_period      = period    # sec
        self.L1_damping     = damping   # dimensionless
        self._log_dict      = {'L1_Nu': 0, 'L1_dist':0, 'L1_TargBrng': 0,
                               'L1_NavBrng': 0, 'L1_XtrackErr': 0,
                               'L1_wca_in': 0, 'L1_wca_out': 0}
        self._par_list      = [key for key in self._log_dict.keys()]
        self._prev_wca_out  = 0
        self._gnd_spd_exp   = 0
        self._dt            = 0

    @staticmethod
    def nav_roll_dem(latAccDem):
        res = np.arctan(latAccDem / 9.81)  # [rad]
        res = np.clip(res, -np.pi / 2, np.pi / 2)
        return res

    def update(self, system, dt, demand, observables):

        cur_leg = demand

        self._dt = dt

        latAccDem = self.update_wp_nav(system, cur_leg[0], cur_leg[1], observables)

        return self.nav_roll_dem(latAccDem)

    def update_wp_nav(self, system, prev_wp, next_wp, observables):

        psi = system.state_dict['S_psi']
        tas = observables['TAS']
        wind = observables['wind']
        gnd_spd_vect = observables['gnd_spd']
        gnd_spd = bn.norm_2d(gnd_spd_vect)
        gnd_trk = bn.wrap_pi(np.arctan2(gnd_spd_vect[0], gnd_spd_vect[1]))

        if gnd_spd < 0.1:
            raise RuntimeError("Unable to fly forward due to wind")
            # APM assumes minimal gnd_spd along heading to keep on computing

        wind_vec = np.array([np.sin(wind[0]), np.cos(wind[0])]) * wind[1]
        tas_vec = np.array([np.sin(psi), np.cos(psi)]) * tas
        current_loc_vec = np.array([system.state_dict['S_x'],
                                    system.state_dict['S_y']])

        K_L1 = 4.0 * self.L1_damping**2
        # use expected ground speed for L1 distance computation
        ref_spd = max(gnd_spd, self._gnd_spd_exp)
        L1_dist = 1/np.pi**2 * self.L1_damping * self.L1_period * ref_spd

        target_bearing = bn.get_bearing(current_loc_vec, next_wp)
        or2target_vec = next_wp - prev_wp                  # AB
        or2target_dist = bn.norm_2d(or2target_vec)
        or2target_vec *= 1./or2target_dist                 # normalize

        origin_vec = current_loc_vec - prev_wp             # A_air
        origin_dist = bn.norm_2d(origin_vec)               # WP_A_dist

        crosstrack_err = -np.cross(origin_vec, or2target_vec)   # A_air % AB_norm (minus due to x-y coord swap)

        coveredTrack_dist = np.dot(origin_vec, or2target_vec)   # alongTrackDist

        Nu1 = 0
        if (origin_dist > L1_dist) and (coveredTrack_dist/np.max([origin_dist, 1.]) < -np.sqrt(2)/2.):
            # calculate Nu to fly to WP A
            origin_vec *= 1./bn.norm_2d(origin_vec)             # normalize
            xtrackVel = -np.cross(gnd_spd_vect, -origin_vec)    # gnd_spd % A_air_norm (minus due to x-y coord swap)
            ltrackVel = np.dot(gnd_spd_vect, -origin_vec)
            Nu = np.arctan2(xtrackVel, ltrackVel)
            nav_bearing = np.arctan2(origin_vec[0], origin_vec[1])
        else:
            # calculate Nu to fly along the AB line
            xtrackVel = -np.cross(gnd_spd_vect, or2target_vec)  # gnd_spd % AB_norm (minus due to x-y coord swap)
            ltrackVel = np.dot(gnd_spd_vect, or2target_vec)
            Nu2 = np.arctan2(xtrackVel, ltrackVel)                  # angle of velocity relative to track line

            sine_Nu1 = crosstrack_err/np.max([L1_dist, 0.1])
            sine_Nu1 = np.clip(sine_Nu1, -np.sqrt(2)/2., np.sqrt(2)/2.)   # capture angle of 45 deg or less
            Nu1 = np.arcsin(sine_Nu1)

            Nu = Nu1 + Nu2
            nav_bearing = bn.get_bearing(prev_wp, next_wp) + Nu1

        # wind corr angle is positive when heading must stay to the right of track
        wind_corr_ang_in = bn.wrap_pi(psi-gnd_trk)
        wind_corr_ang_out, gnd_spd_out = bn.get_wind_corr(nav_bearing, tas, wind_obs=wind)[0:2]
        # Note: when using direct prev_wp to next_wp bearing instead of Nu1 corrected nav_bearing
        # the downwind performance is improved but everything else is degraded

        self._gnd_spd_exp = gnd_spd_out

        apply_wind_correction = True
        if apply_wind_correction:
            # TODO: test the following
            # - apply the correction only if the wca_out is constant or changing slowly (wca_out ~ Nu1)
            # OR
            # - apply the correction only if the wca_out is changing considerably (wca_out ~ Nu1)
            force_basic_correction = True
            has_low_dynamics = (abs(wind_corr_ang_out - self._prev_wca_out) > 0.05*self._dt) # 0.05 rad/sec (0.6 deg/sec)
            high_capture_angle_supported = False
            if force_basic_correction or has_low_dynamics:
                Nu = Nu + wind_corr_ang_out - wind_corr_ang_in
            elif high_capture_angle_supported and (abs(Nu1) > 0.01*0.2):
                wind_corr_ang_out = bn.get_wind_corr(bn.get_bearing(prev_wp, next_wp), tas, wind_obs=wind)[0]
                Nu = Nu + wind_corr_ang_out - wind_corr_ang_in
                # option: fade-out this correction as the capture angle becomes smaller
            else:
                wind_corr_ang_in = 0

            self._prev_wca_out = wind_corr_ang_out

        # TODO How to anticipate the future rate of change of the demand??
        # We must adjust the trajectory in a manner that ensures that
        # the future (short term) rate of change of the demand will not exceed airplane limits.
        #   L1_Period does this I guess

        # TODO Consider adapting L1_distance to the expression abs(wca_out - wca_in)

        # TODO RUN PARALLEL SIMULATIONS FOR MODIFIED AND BASE L1 CONTROLLERS

        Nu = np.clip(Nu, -np.pi/2, np.pi/2)

        latAccDem = K_L1 * gnd_spd**2 * np.sin(Nu) / L1_dist

        self._log_dict['L1_Nu'] = Nu        # bearing_error
        self._log_dict['L1_dist'] = L1_dist
        self._log_dict['L1_TargBrng'] = target_bearing
        self._log_dict['L1_NavBrng'] = nav_bearing
        self._log_dict['L1_XtrackErr'] = crosstrack_err
        self._log_dict['L1_wca_in'] = wind_corr_ang_in
        self._log_dict['L1_wca_out'] = wind_corr_ang_out

        return latAccDem


stdRollReg = RollRegulator()
stdNavReg = L1NavRegulator()

