import numpy as np

try:
    from aircraft.aircraft import basic_aircraft
    from models.system import System
    from models.env import Env
    from sim import Sim
except:
    print "Check run path, must be run from: ./40.SmoothTraj"

import matplotlib.pyplot as plt

phi_init = 0.5

sys = System(phi=phi_init, psi=0, use_jac=True)
env = Env(150, 1)

t_sim_end = 40      # sec
sampling_rate = 100 # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

sim = Sim(basic_aircraft, sys, env, time, verbose=False)
sim.override_roll(phi_init)
sim.run_simulation()

plt.plot(sim.logs['x'][:-2], sim.logs['y'][:-2])

plt.show()