import numpy as np
import sys


class Sim(object):

    def __init__(self, aircraft, system, env, time=None, verbose=False):

        self._aircraft  = aircraft
        self._system    = system
        self._env       = env
        self._time      = time
        self._time_step = 0
        self._log_dict  = {}
        self._par_list  = [[]]
        self._verbose   = verbose
        self._only_low_level    = False
        self._override_roll_dem = 0

        self.setup_logging()

    def tick(self, dt):

        self.save_current_par_dict()
        self._system.propagate(self._aircraft, self._env, dt)
        self._aircraft.measure(self._system.observables)
        self._aircraft.update(self._system, dt, self._only_low_level, self._override_roll_dem)
        self._time_step += 1

    def run_simulation(self):

        if type(self._time) is not np.ndarray:
            raise TypeError('Incorrect time format')

        for ii, t in enumerate(self._time[1:]):
            dt = t - self._time[ii]
            self.tick(dt)

            if self._verbose:
                sys.stdout.write('. ')

    def override_roll(self, roll_dem):
        self._only_low_level = True
        self._override_roll_dem = roll_dem

    def setup_logging(self):
        self._par_list[0] = ['x', 'y', 'phi', 'psi', 'p']
        for key in self._aircraft.regulators:
            self._par_list.append(self._aircraft.regulators[key].par_list)

        for sublist in self._par_list:
            for name in sublist:
                self._log_dict[name] = np.empty_like(self._time)  # uninitialized (arbitrary data)

                if self._verbose:
                    sys.stdout.write(name + str(' '))
        print  # end line

    def save_current_par_dict(self):

        for name in self._par_list[0]:
            self._log_dict[name][self._time_step] = self._system.state_dict[name]

        for idx, key in enumerate(self._aircraft.regulators):
            for subkey in self._par_list[idx + 1]:
                self._log_dict[subkey][self._time_step] = self._aircraft.regulators[key].current_val_dict[subkey]

    @property
    def logs(self):
        return self._log_dict

