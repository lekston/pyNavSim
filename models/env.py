import numpy as np
import utils.basic_nav as bn


class Env:

    def __init__(self, wind_dir_to=0, wind_spd=0):

        self._wind_to = np.array([wind_dir_to, wind_spd])
        self._observables = {'wind': self._wind_to}

    @property
    def wind(self):
        return self._wind_to

    @property
    def wind_to(self):
        return self._wind_to

    @property
    def wind_from(self):
        dir_to = self.wind[0]
        spd = self.wind[1]
        return np.array([bn.wrap_2pi(dir_to - np.pi), spd])

    @property
    def observables(self):
        return self._observables

    def print_wind(self):
        print("Wind_dir_to: %d / speed: %d" % (self._wind_to[0], self._wind_to[1]))
