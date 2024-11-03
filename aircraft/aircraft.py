from .regulator import stdRollReg, stdNavReg
from .flightPlan import basic_fpl

import numpy as np


class Aircraft(object):

    def __init__(self, flightPlan=basic_fpl, roll_reg=stdRollReg, nav_reg=stdNavReg, tas=15):
        self._TAS = tas  # m/s
        self._fpl = flightPlan
        self._regulators = dict(roll=roll_reg, nav=nav_reg)
        self._last_controls = np.array([0])  # zero control input is still a valid input
        self._par_list = []
        self._obs_data = dict()

    @property
    def airspeed(self):
        return self._TAS

    @property
    def regulators(self):
        return self._regulators

    @property
    def controls(self):
        return self._last_controls

    @property
    def flight_plan(self):
        return self._fpl

    def reset_fpl(self):
        self._fpl.reset_fpl()

    def measure(self, obs_dict):
        self._obs_data = obs_dict

    def update(self, system, dt, only_low_level=False, override_roll_dem=0):

        # pass position to fpl object to verify reaching waypoints
        cur_leg = self._fpl.update(system.state[:2])
        roll_dem = self._regulators['nav'].update(system, dt, cur_leg, self._obs_data)

        if only_low_level:
            roll_dem = override_roll_dem

        p_dot_cmd = self._regulators['roll'].update(system, dt, roll_dem, self._obs_data)
        self._last_controls = np.array([p_dot_cmd])

        return p_dot_cmd

basic_aircraft = Aircraft()


