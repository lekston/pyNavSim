import numpy as np
import sys


class Sim(object):

    def __init__(self, aircraft, system, env, time=None):

        self._aircraft  = aircraft
        self._system    = system
        self._env       = env
        self._time      = time
        self._log_dict  = {}
        self._par_list  = []

        self.setup_logging()

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

    def setup_logging(self):
        self._par_list = ['x', 'y', 'phi', 'psi', 'p']
        for key in self._aircraft.regulators:
            self._par_list.extend(self._aircraft.regulators[key].par_list)

        for name in self._par_list:
            sys.stdout.write(name + str(' '))

    def save_current_par_dict(self):
        pass
