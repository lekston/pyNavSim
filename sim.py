import numpy as np
import sys


class Sim(object):

    def __init__(self, aircraft, system, env):

        self._aircraft  = aircraft
        self._system    = system
        self._env       = env

    def time_step(self, dt):

        self._system.propagate(self._aircraft, self._env, dt)
        self._aircraft.update(self._system)

    def run_simulation(self, time=None, verbose=False):

        if time is None:
            raise ValueError("Must set time first")

        for ii, t in enumerate(time[1:]):
            dt = t - time[ii]
            if verbose and True:
                sys.stdout.write('. ')

        self.save_current_par_dict()

    def save_current_par_dict(self):
        pass
