import numpy as np
from scipy.integrate import ode
from models.flat_earth import kin_equats, kin_equats_jac


class System(object):

    # noinspection PyPep8Naming
    def __init__(self, phi=0, psi=0, tas=15,
                 integrator='dopri5', use_jac=False, **integrator_params):

        self.x = 0
        self.y = 0
        self.p = 0
        self._state = np.array([self.x, self.y, phi, psi, self.p])
        self._TAS = tas

        self.kin_equats = kin_equats

        if use_jac:
            jac_kin = kin_equats_jac
        else:
            jac_kin = None

        self._ode_kin_equats = ode(kin_equats, jac=jac_kin)
        self._ode_kin_equats.set_integrator(integrator, **integrator_params)
        self._ode_kin_equats.set_initial_value(y=self._state)

    @property
    def state(self):
        return self._state

    def propagate(self, aircraft, env, dt=0.01):

        time = self._ode_kin_equats.t + dt

        self._ode_kin_equats.set_f_params(self._TAS, env.wind, aircraft.controls)
        state = self._ode_kin_equats.integrate(time)

        if self._ode_kin_equats.successful():
            self._state = state
        else:
            raise RuntimeError('Integration was not successful')

        return self._state

