# use any example first to define globals

t_sim_end = 80       # sec
sampling_rate = 100  # Hz
f = sampling_rate
N = t_sim_end * sampling_rate + 1
time = np.linspace(0, t_sim_end, num=N)

w_spd = 10

# at phi = 45deg one full orbit takes about 10.6

X = 10*time + ac_tas*10.6/(2*np.pi)*np.sin(2*np.pi*time/10.6)
Y = -ac_tas*10.6/(2*np.pi)*np.cos(2*np.pi*time/10.6)

fig6 = plt.figure()
plt.plot(X[:-2],Y[:-2],X[:-2:2*f],Y[:-2:2*f],'gx')
plt.grid()
ax1 = fig6.gca()
dr.set_1to1_scale(ax1)

