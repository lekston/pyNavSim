# PREREQUSITES
sudo apt install cm-super

# HOW TO START?

- launch ipython console from root
- call:
```
    from runpy import run_path
    run_path('./examples/ex_00.py')
```

- for restaring the simulation:
    `basic_aircraft.reset_fpl()` # (ex_00)
        or
    `ac.reset_fpl()`             # (ex_01)

# ACCESS to LOGGED DATA:

- after first run of the simulation, log data area available via:
    `sim.log['name_of_key']`

- to check currenly available keys run:
```
    for key in sim.logs.keys():
        print(key)
```

# LOGGING DATA NAMING CONVENTION:
 - S_ prefix - system state vector variables
 - R_ prefix - roll regulator
 - L1_ prefix - L1 navigation regulator

# COORDINATE FRAME CONVENTION

Sim coordinate frame convention:
 - Y is North
 - X is East
 - phi is measured from Y to the right (normal heading convention)
 - this simplifies the use of plotting modules

APM coordinate frame convention:
 - NED:
    - X is North
    - Y is East
    - Z is Down

# DEMO outputs

Visualization of simulator states from example `ex_02_into_wind_customRegs.py` is shown below.

![Demo Image](./pics/demo_smoooth_traj.png)