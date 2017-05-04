from numpy import sin, cos, tan
import numpy as np

g = 9.81

""" Kinematic equations for horizontal coordinated turns

    Conventions:
    - wind_dir - where to the wind is blowing
"""


def kin_equats(time, state, TAS, wind):
    # TODO must provide 'p' - either as a state or control variable
    #   It is simpler to have it as control variable and keeps
    #   its state separately

    phi = state[3]
    psi = state[4]

    wind_dir, wind_spd = wind

    dx_dt = cos(psi) * TAS + cos(wind_dir) * wind_spd

    dy_dt = sin(psi) * TAS + sin(wind_dir) * wind_spd

    dphi_dt = p # TODO must have low level roll rate controller

    dpsi_dt = g * tan(phi) / TAS

    return np.array([dx_dt, dy_dt, dphi_dt, dpsi_dt])


def kin_equats_jac(time, state, TAS):

    phi = state[3]
    psi = state[4]

    jac = np.zeros([4,4])

    jac[0,3] = -sin(psi) * TAS
    jac[1,3] = cos(psi) * TAS
    jac[3,2] = g / (cos(phi)**2 * TAS)

    return jac