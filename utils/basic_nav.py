""" Conventions & units:
    - All angles in radians (unless stated otherwise)
    - positive crosswind is to the right (blows along Y axis)
    - positive tailwind increases ground speed (blows along X axis)
    - negative tailwind is headwind
"""
from numpy import sin, cos, arcsin
import numpy as np

''' wind_dir_to - where TO the wind is blowing '''


def get_wind_corr(track_angle, tas, wind_obs):
    wind_dir_to, wind_spd = wind_obs
    wta = wrap_pi(wind_dir_to - track_angle)  # wind dir relative to track angle

    xwind = wind_spd * sin(wta)
    twind = wind_spd * cos(wta)

    wca = arcsin(-xwind / tas)
    gnd_spd = tas * cos(wca) + twind

    return np.array([wca, gnd_spd, xwind, twind])


def get_gnd_spd(psi, tas, wind_obs):
    
    wind_dir_to, wind_spd = wind_obs
    
    v_x = sin(psi) * tas + sin(wind_dir_to) * wind_spd
    v_y = cos(psi) * tas + cos(wind_dir_to) * wind_spd
    return np.array([v_x, v_y])


def get_bearing(origin, dest):
    """
    :param origin:
    :param dest:
    :return: ground course from origin to destination
    """
    dx = dest[0] - origin[0]
    dy = dest[1] - origin[1]
    return np.arctan2(dx, dy)


def dist_2d(x1, x2):
    assert(type(x1) in [tuple, list, np.ndarray])
    assert(type(x2) in [tuple, list, np.ndarray])
    diff = (x1[0] - x2[0], x1[1] - x2[1])
    return norm_2d(diff)


def norm_2d(x):
    assert(type(x) in [tuple, list, np.ndarray])
    return np.sqrt(x[0]**2 + x[1]**2)


def wrap_180(angle):
    result = wrap_360(angle)
    if result > 180.0:
        result -= 360.0
    return result


def wrap_360(angle):
    return angle % 360.0


def wrap_2pi(angle):
    return angle % (2*np.pi)


def wrap_pi(angle):
    result = wrap_2pi(angle)
    if result > np.pi:
        result -= 2*np.pi
    return result
