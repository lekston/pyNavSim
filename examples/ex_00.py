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


phi_init = 0.0

sys = System(phi=phi_init, psi=0, use_jac=True)
env = Env(wind_dir_to=np.deg2rad(270), wind_spd=5)

t_sim_end = 60       # sec
sampling_rate = 100  # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

sim = Sim(basic_aircraft, sys, env, time, verbose=False)
#sim.override_roll(np.deg2rad(40))
sim.run_simulation()


fig1 = plt.figure(1)
plt.plot(sim.logs['x'][:-2], sim.logs['y'][:-2])
# plt.plot(time[:-2], sim.logs['x'][:-2])
ax1 = fig1.gca()
dr.set_1to1_scale(ax1)

# TODO fig2 & fig3 to subplots
fig2 = plt.figure(2)
plt.plot(time, sim.logs['p'], 'b-', time, sim.logs['p_dem'], 'r-')
ax2 = fig2.gca()
# set_1to1_scale(ax2)
ax2.set_ylabel('p[rad/s]')

fig3 = plt.figure(3)
plt.plot(time, sim.logs['phi'], 'b-', time, sim.logs['phi_dem_ll'], 'r-')
ax3 = fig3.gca()
# set_1to1_scale(ax3)
ax3.set_ylabel('phi[rad]')

plt.show()

