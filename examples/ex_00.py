import numpy as np

try:
    from aircraft.aircraft import basic_aircraft
    from models.system import System
    from models.env import Env
    from utils import drawing as dr
    from sim import Sim
except:
    print "Check run path, must be run from: ./40.SmoothTraj"

import matplotlib.pyplot as plt

plt.close("all")
plt.rc('text', usetex=True)
plt.rc('font', family='serif')

phi_init = 0.0

sys = System(phi=phi_init, psi=0, use_jac=True)
env = Env(wind_dir_to=np.deg2rad(270), wind_spd=5)

t_sim_end = 80       # sec
sampling_rate = 100  # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

sim = Sim(basic_aircraft, sys, env, time, verbose=False)
# sim.override_roll(np.deg2rad(40))
sim.run_simulation()


fig1 = plt.figure()
plt.plot(sim.logs['S_x'][:-2], sim.logs['S_y'][:-2])
# plt.plot(time[:-2], sim.logs['x'][:-2])
plt.grid()
ax1 = fig1.gca()
plt.title("Trajectory")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
dr.set_1to1_scale(ax1)

fig2 = plt.figure()
ax1 = plt.subplot(2, 1, 1)
plt.plot(time, sim.logs['S_p'], time, sim.logs['R_p_dem'], 'r')
plt.xlabel("Time [s]")
plt.ylabel("$p\left[\\frac{rad}{s}\\right]$")
plt.legend(["$p$", "$p_{dem}$"])
plt.grid()

ax2 = plt.subplot(2, 1, 2, sharex=ax1)
plt.plot(time, sim.logs['S_phi'], time, sim.logs['R_phi_dem_ll'], 'r')
plt.xlabel("Time [s]")
plt.ylabel("$\phi [rad]$")
plt.legend(["$\phi$", "$\phi_{dem}$"])
plt.grid()

plt.show()
