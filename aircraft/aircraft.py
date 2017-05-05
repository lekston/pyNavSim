from regulator import rollReg, navReg
from flightPlan import basic_fpl

import numpy as np


class Aircraft(object):

    def __init__(self, flightPlan=basic_fpl):
        self._TAS = 15  # m/s
        self._fpl = flightPlan
        self._regulators = dict(roll=rollReg, nav=navReg)
        self._last_controls = np.array([0])  # zero control input is still a valid input
        self._par_list = []

    @property
    def airspeed(self):
        return self._TAS

    @property
    def regulators(self):
        return self._regulators

    @property
    def controls(self):
        return self._last_controls

    def update(self, system, only_low_level=False, override_roll_dem = 0):

        # pass position to fpl object to verify reaching waypoints
        cur_leg = self._fpl.update(system.state[:2])

        roll_dem      = self._regulators['nav'].update(system, cur_leg)
        if only_low_level:
            roll_dem = override_roll_dem
        roll_rate_dem = self._regulators['roll'].update(system, roll_dem)

        self._last_controls = np.array([roll_rate_dem])

        # TODO logging of aircraft control variables

        return roll_rate_dem

basic_aircraft = Aircraft()


