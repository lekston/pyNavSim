from numpy import sin, cos, tan
import numpy as np

g = 9.81

""" Kinematic equations for horizontal coordinated turns

    Conventions:
    - wind_dir - where to the wind is blowing
"""

def kin_equats():

    dx_dt = cos(psi) * TAS + cos(wind_dir) * wind_spd

    dy_dt = sin(psi) * TAS + sin(wind_dir) * wind_spd

    dphi_dt = p # TODO must have low level roll rate controller

    dpsi_dt = g * tan(phi) / TAS

    return np.array([dx_dt, dy_dt, dphi_dt, dpsi_dt])