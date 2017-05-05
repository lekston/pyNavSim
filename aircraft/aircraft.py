from regulator import rollReg, navReg


class Aircraft(object):

    def __init__(self):
        self._TAS = 15  # m/s
        self._regulators = dict(roll=rollReg,
                                nav=navReg)

    @property
    def airspeed(self):
        return self._TAS

    def update(self, system, demand):
        for key in self._regulators:
            self._regulators[key].update(system, demand)

