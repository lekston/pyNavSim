import numpy as np

try:
    from aircraft.aircraft import Aircraft
    from aircraft.flightPlan import FlightPlan
    from models.system import System
    from models.env import Env
    from utils import drawing as dr
    from sim import Sim
except Exception as e:
    print "Check run path, must be run from: ./40.SmoothTraj"
    print type(e)
    print e.args
    print e

import matplotlib.pyplot as plt

plt.close("all")
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

phi_init = 0.0

sys = System(phi=phi_init, psi=0, use_jac=True)
env = Env(wind_dir_to=np.deg2rad(270), wind_spd=5)
env.print_wind()

t_sim_end = 80       # sec
sampling_rate = 100  # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

'''Define flight plan'''
mirror = True

if mirror:
    wpt_list = np.array([0.,     100.,
                         0.,     200.,
                        -200.,   0.,
                        -200.,   400.,
                         200.,   400.])
else:
    wpt_list = np.array([0.,     100.,
                         0.,     200.,
                         200.,   0.,
                         200.,   400.,
                        -200.,   400.])

wpt_arr = np.ndarray(shape=(5, 2), dtype=float, buffer=wpt_list)

fpl = FlightPlan(wpt_arr)
ac = Aircraft(fpl)

sim = Sim(ac, sys, env, time, verbose=False)
# sim.override_roll(np.deg2rad(40))
sim.run_simulation()


fig1 = plt.figure()
wp = ac.flight_plan.wpt_arr
plt.plot(sim.logs['S_x'][:-2], sim.logs['S_y'][:-2], wp[:,0], wp[:,1], 'ro', wp[:,0], wp[:,1], 'r')
# plt.plot(time[:-2], sim.logs['x'][:-2])
plt.grid()
ax1 = fig1.gca()
plt.title("Trajectory")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
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

plt.show()
