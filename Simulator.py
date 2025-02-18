import pandas as pd
import matplotlib as mpl
import Controller

df = pd.read_csv("data.csv")
df = df.dropna(subset=['gps.latitude'])
df = df.drop(df['gps.latitude'] == 0)
df_lat = df[df["gps.latitude"]]
df_lon = df[df["gps.longitude"]]
df_alt = df[df["gps.altitude"]]
df_time = df[df["timestamp"]]

turret_lat = 0
turret_lon = 0

control = Controller(turret_lat, turret_lon, 0, df_lat[0], df_lon[0], df_alt[0], df_time[0])

for i, row in df.iterrows:
    yaw, pitch = control.controller(df_lat[i], df_lon[i], df_alt[i], df_time[i])
    df['Yaw_Error'] = yaw
    df['Pitch_Error'] = pitch

df_sample = df.iloc[::100]

df_sample.plot.scatter(x='timestamp', y='Yaw_Error')