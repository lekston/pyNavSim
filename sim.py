import numpy as np
import sys


class Sim(object):

    def __init__(self, aircraft, system, env, time=None):

        self._aircraft  = aircraft
        self._system    = system
        self._env       = env
        self._time      = time

    def time_step(self, dt):

        self._system.propagate(self._aircraft, self._env, dt)
        self._aircraft.update(self._system)

    def run_simulation(self, verbose=False):

        if type(self._time) is not np.ndarray:
            raise TypeError('Incorrect time format')

        for ii, t in enumerate(self._time[1:]):
            dt = t - self._time[ii]
            self.time_step(dt)

            if verbose:
                sys.stdout.write('. ')

        self.save_current_par_dict()

    def set_par_dict(self, par_list):
        pass

    def save_current_par_dict(self):
        pass
