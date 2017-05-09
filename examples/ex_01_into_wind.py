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
    print "Check run path, must be run from: ./40.SmoothTraj"

import matplotlib.pyplot as plt

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

fig1 = plt.figure(1)
plt.plot(sim.logs['S_x'][:-2], sim.logs['S_y'][:-2])
# plt.plot(time[:-2], sim.logs['x'][:-2])
ax1 = fig1.gca()
dr.set_1to1_scale(ax1)

# TODO fig2 & fig3 to subplots
fig2 = plt.figure(2)
plt.plot(time, sim.logs['S_p'], 'b-', time, sim.logs['R_p_dem'], 'r-')
ax2 = fig2.gca()
# set_1to1_scale(ax2)
ax2.set_ylabel('p[rad/s]')

fig3 = plt.figure(3)
plt.plot(time, sim.logs['S_phi'], 'b-', time, sim.logs['R_phi_dem_ll'], 'r-')
ax3 = fig3.gca()
# set_1to1_scale(ax3)
ax3.set_ylabel('phi[rad]')

fig4 = plt.figure(4)
plt.plot(time, [ bn.wrap_180(np.rad2deg(psi)) for psi in sim.logs['S_psi']])
ax4 = fig4.gca()
ax4.set_ylabel('psi[deg]')

plt.show()