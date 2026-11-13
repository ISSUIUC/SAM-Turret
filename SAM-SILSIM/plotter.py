import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
lat_origin = 41.486532 # sam base 
lon_origin = -89.500903

df_rocket = pd.read_csv('./data/midas_sustainer_trimmed.csv')
df_turret = pd.read_csv('./output/turret_silsim_output.csv')

r_lat = df_rocket['gps.latitude'] / 1e7
r_lon = df_rocket['gps.longitude'] / 1e7
r_alt = df_rocket['kalman.position.px']

r_east = (r_lon - lon_origin) * 111000.0 * np.cos(np.radians(lat_origin)) # meters relative to position of sam 
r_north = (r_lat - lat_origin) * 111000.0
r_up = r_alt  # Altitude in meters


t_east = df_turret['norm_east'] 
t_north = df_turret['norm_north'] 
t_up = df_turret['norm_up'] 

fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(projection='3d')

n_frames = len(df_turret)
rocket_indices = np.linspace(0, len(r_east) - 1, n_frames).astype(int) # frame sync

r_east_f = r_east.iloc[rocket_indices].values
r_north_f = r_north.iloc[rocket_indices].values
r_up_f = r_up.iloc[rocket_indices].values

t_east_f = t_east.values
t_north_f = t_north.values
t_up_f = t_up.values


def anim():
  ax.set_xlim(min(r_east.min(), -500), max(r_east.max(), 500) + 1000)
  ax.set_ylim(min(r_north.min(), -500), max(r_north.max(), 500) + 1000)
  ax.set_zlim(0, max(r_up.max(), t_up.max()) + 500)
  ax.set_xlabel('East (meters)')
  ax.set_ylabel('North (meters)')
  ax.set_zlabel('Up (meters)')
  ax.set_title('SAMMY SILSIM')

  ax.plot(r_east,r_north,r_up,color='blue',alpha=0.3,lw=1,label='Full Rocket Path')
  ax.scatter([0], [0], [0], color='red', s=150, marker='^', label='SAM Origin')
  return []


def update_anim(num):
  ax.clear()
  anim()

  ax.quiver(0,0,0,t_east_f[num] * 100000,t_north_f[num]* 100000,t_up_f[num]* 100000,color='orange',arrow_length_ratio=0.01,alpha=0.8,label='Turret POINT') 

  ax.scatter([r_east_f[num]],[r_north_f[num]],[r_up_f[num]],color='purple',s=120,marker='o',label='Rocket Position',depthshade=False)

  ax.legend(loc='upper left')
  return (anim,)


# frames
skip = 5
ani = animation.FuncAnimation(
    fig,
    update_anim,
    frames=range(0, n_frames, skip),
    init_func=anim,
    interval=30,
    blit=False,
)
plt.show()
