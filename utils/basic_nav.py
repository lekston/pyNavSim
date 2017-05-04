""" Conventions & units:
    - bearing and wind_dir are in degrees
    - positive crosswind is to the right (blows along Y axis)
    - positive tailwind increases ground speed (blows along X axis)
    - negative tailwind is headwind

struct DR_Results {
    float WindCorrAngle;
    float GndSpd;
    float CrossWind;
    float TailWind;
}; // Dead Reckon results

struct DR_Results get_wind_corr(float bearing, float airspeed, float wind_dir, float wind_spd)
{
    struct DR_Results results;

    float wta = ToRad(wrap_180(bearing - wind_dir)); // wind dir to track angle

    results.CrossWind = wind_spd * sinf(wta);
    results.TailWind = -wind_spd * cosf(wta);

    float wca = -asinf(results.CrossWind / airspeed); // radians
    results.WindCorrAngle = ToDeg(wca);
    results.GndSpd = airspeed * cosf(wca) + results.TailWind;

    return results;
}
"""
from numpy import sin, cos, arcsin
import numpy as np

''' wind_dir - where to the wind is blowing '''


def get_wind_corr(psi, TAS, wind_dir, wind_spd):
    wta = wrap_180(wind_dir - psi) # wind dir relative to track angle

    xwind = wind_spd * sin(wta)
    twind = wind_spd * cos(wta)

    wca = arcsin(-xwind / TAS)
    gnd_spd = TAS * cos(wca) + twind

    return np.array([wca, gnd_spd, xwind, twind])


def wrap_180(angle):
    result = wrap_360(angle)
    if result > 180.0:
        result -= 360.0
    return result


def wrap_360(angle):
    return angle % 360.0
