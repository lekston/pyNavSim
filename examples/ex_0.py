import numpy as np

try:
    from aircraft.aircraft import basic_aircraft
    from models.system import System
    from models.env import Env
    from sim import Sim
except:
    print "Check run path, must be run from: ./40.SmoothTraj"

sys = System()
env = Env(150, 10)

t_sim_end = 20      # sec
sampling_rate = 100 # Hz
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

sim = Sim(basic_aircraft, sys, env, time)
sim.run_simulation(verbose=False)

