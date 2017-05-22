import numpy as np
import utils.basic_nav as bn
from scipy.integrate import ode
from models.flat_earth import kin_equats, kin_equats_jac


class System(object):

    # noinspection PyPep8Naming
    def __init__(self, x=0, y=0, phi=0, psi=0, tas=15,
                 integrator='dopri5', use_jac=False, **integrator_params):

        self.x = x
        self.y = y
        self.p = 0.
        self._state = np.array([self.x, self.y, phi, psi, self.p])
        self._TAS = tas
        self._observables = {'gnd_spd': 0.,
                             'TAS': tas}

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

    @property
    def state_dict(self):
        res = dict(S_x=self._state[0], S_y=self._state[1],
                   S_phi=self._state[2], S_psi=self._state[3], S_p=self._state[4])
        return res

    @property
    def observables(self):
        return self._observables

    def propagate(self, aircraft, env, dt=0.01):

        time = self._ode_kin_equats.t + dt

        self._ode_kin_equats.set_f_params(aircraft.airspeed, env.wind, aircraft.controls)
        state = self._ode_kin_equats.integrate(time)

        if self._ode_kin_equats.successful():
            self._state = state
        else:
            raise RuntimeError('Integration was not successful')

        self._update_observables(aircraft, env)

        return self._state

    def _update_observables(self, aircraft, env):
        gnd_spd = bn.get_gnd_spd(self.state_dict['S_psi'], self._TAS, env.wind_to)
        self._observables['gnd_spd'] = gnd_spd
        self._observables['TAS'] = self._TAS
        self._observables.update(env.observables)
