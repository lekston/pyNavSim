import numpy as np


class Env:

    def __init__(self, wind_dir=0, wind_spd=0):

        self._wind = np.array([wind_dir, wind_spd])

    @property
    def wind(self):
        return self._wind
