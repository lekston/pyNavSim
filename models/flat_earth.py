from numpy import sin, cos, tan
import numpy as np

g = 9.81

""" Kinematic equations for horizontal coordinated turns

    Conventions:
    - wind_dir - where to the wind is blowing
"""


def kin_equats(time, state, tas, wind, controls):
    # TODO must provide 'p' - either as a state or control variable
    #   It is simpler to have it as control variable and keeps
    #   its state separately

    phi = state[2]
    psi = state[3]
    p   = state[4]

    rollRateDem = controls[0]

    wind_dir, wind_spd = wind

    dx_dt = cos(psi) * tas + cos(wind_dir) * wind_spd

    dy_dt = sin(psi) * tas + sin(wind_dir) * wind_spd

    dphi_dt = p  # TODO must have low level roll rate regulator

    dpsi_dt = g * tan(phi) / tas

    dp_dt = rollRateDem

    return np.array([dx_dt, dy_dt, dphi_dt, dpsi_dt, dp_dt])


def kin_equats_jac(time, state, tas):

    phi = state[2]
    psi = state[3]

    jac = np.zeros([4, 4])

    jac[0, 3] = -sin(psi) * tas
    jac[1, 3] = cos(psi) * tas
    jac[2, 2] = 1
    jac[3, 2] = g / (cos(phi)**2 * tas)

    return jac
