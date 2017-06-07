import numpy as np
import scipy.linalg as la

try:
    from models.system import System
    from models.env import Env
except:
    print "Error importing models"

try:
    from aircraft.aircraft import Aircraft
except:
    print "Error importing aircraft"

try:
    from aircraft.flightPlan import FlightPlan
    from aircraft.regulator import RollRegulator, L1NavRegulator
    from utils import drawing as dr
    from utils import basic_nav as bn
    from sim import Sim
except:
    print "Check run path, must be run from: ./40.SmoothTraj"

import matplotlib.pyplot as plt

plt.close("all")
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

phi_init = 0.0

t_sim_end = 60       # sec
sampling_rate = 100  # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

''' Define flight plan as below '''
wpt_list = np.array([0.,     0.,
                     0.,     80.,
                     -20.,    40.,
                     -20.,    300.,
                     200.,   300.])
wpt_listB = np.array([0.,     0.,
                     0.,     80.,
                     20.,    40.,
                     20.,    300.,
                     -200.,   300.])
wpt_arr = np.ndarray(shape=(5, 2), dtype=float, buffer=wpt_list)

''' Define simulation components '''
ac_tas = 15.
env = Env(wind_dir_to=np.deg2rad(90), wind_spd=12)
init = bn.get_wind_corr(track_angle=np.deg2rad(0), tas=ac_tas, wind_obs=env.wind_to)
sys = System(phi=phi_init, psi=init[0], use_jac=True)

fpl = FlightPlan(wpt_arr, accept_dist=60)
rreg = RollRegulator(phi_max=42)    # default phi_max is 35
nreg = L1NavRegulator(period=20)    # default period is 30
ac = Aircraft(fpl, roll_reg=rreg, nav_reg=nreg, tas=ac_tas)

sim = Sim(ac, sys, env, time, verbose=False)
#sim.override_roll(np.deg2rad(40))
sim.run_simulation()

fig1 = plt.figure()
wp = ac.flight_plan.wpt_arr
plt.plot(sim.logs['S_x'][:-2], sim.logs['S_y'][:-2], wp[:, 0], wp[:, 1], 'ro')
# plt.plot(time[:-2], sim.logs['x'][:-2])
plt.grid()
plt.title("Trajectory")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
ax1 = fig1.gca()
dr.set_1to1_scale(ax1)

fig2 = plt.figure()
ax1 = plt.subplot(2, 1, 1)
plt.plot(time, sim.logs['S_p'], time, sim.logs['R_p_dot_cmd'], 'r')
plt.xlabel("Time [s]")
plt.ylabel("$p\left[\\frac{rad}{s}\\right]$")
plt.legend(["$p$", "$\dot{p}_{cmd}$"])
plt.grid()

ax2 = plt.subplot(2, 1, 2, sharex=ax1)
plt.plot(time, sim.logs['S_phi'], time, sim.logs['R_phi_dem_ll'], 'r')
plt.xlabel("Time [s]")
plt.ylabel("$\phi [rad]$")
plt.legend(["$\phi$", "$\phi_{dem}$"])
plt.grid()

fig3 = plt.figure()
gnd_spd_vect = [(bn.get_gnd_spd(psi, ac_tas, env.wind_to)) for psi in sim.logs['S_psi']]
gnd_spd = [la.norm(vect) for vect in gnd_spd_vect]
gnd_track = [bn.wrap_180(np.rad2deg(np.arctan2(vect[0], vect[1]))) for vect in gnd_spd_vect]
plt.plot(time, gnd_spd,
         time, sim.logs['L1_XtrackErr'])
plt.xlabel("Time [s]")
#plt.ylabel("$V_{E}$ [$m/s$]")
plt.legend(["$V_{E}$ [$m/s$]", "$XtrackErr$ [$m$]"])
plt.grid()

fig4 = plt.figure()
plt.plot(time, [bn.wrap_180(np.rad2deg(psi)) for psi in sim.logs['S_psi']],
         time, gnd_track)
plt.xlabel("Time [s]")
#plt.ylabel("$\psi$ [$^\circ$]")
plt.legend(["$\psi$ [$^\circ$]", "$\psi_{gnd}$ [$^\circ$]"])
plt.grid()

fig5 = plt.figure()
plt.plot(time, sim.logs['L1_wca_in'], time, sim.logs['L1_wca_out'])
plt.xlabel("Time [s]")
plt.legend(["$wca_{in}$ [$rad$]", "$wca_{out}$ [$rad$]"])
plt.grid()

plt.show()
