import numpy as np

try:
    from aircraft.aircraft import Aircraft
    from aircraft.flightPlan import FlightPlan
    from models.system import System
    from models.env import Env
    from utils import drawing as dr
    from utils import basic_nav as bn
    from sim import Sim
except:
    print("Check run path, must be run from: ./40.SmoothTraj")

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
wpt_list = np.array([20.,     0.,
                     20.,     800.])
wpt_arr = np.ndarray(shape=(2, 2), dtype=float, buffer=wpt_list)

''' Define simulation components '''
sys = System(phi=phi_init, psi=0, use_jac=True)
env = Env(wind_dir_to=np.deg2rad(120), wind_spd=13)
fpl = FlightPlan(wpt_arr)
ac = Aircraft(fpl)

''
sim = Sim(ac, sys, env, time, verbose=False)
#sim.override_roll(np.deg2rad(40))
sim.run_simulation()

fig1 = plt.figure()
wp = ac.flight_plan.wpt_arr
plt.plot(sim.logs['S_x'][:-2], sim.logs['S_y'][:-2], wp[:,0], wp[:,1], 'ro', wp[:,0], wp[:,1], 'r')
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

fig4 = plt.figure()
plt.plot(time, [bn.wrap_180(np.rad2deg(psi)) for psi in sim.logs['S_psi']])
plt.xlabel("Time [s]")
plt.ylabel('$\psi$ [$^\circ$]')
plt.grid()

plt.show()
