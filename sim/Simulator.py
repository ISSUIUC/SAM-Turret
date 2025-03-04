import pandas as pd
import matplotlib as mpl
import Controller

df_clean = pd.read_csv("df_clean.csv")   # read from cleaned (smaller) file

# df_lat = df[df["gps.latitude"]]
# df_lon = df[df["gps.longitude"]]
# df_alt = df[df["gps.altitude"]]
# df_time = df[df["timestamp"]]

turret_lat = 0
turret_lon = 0

control = Controller(turret_lat, turret_lon, 0, df_clean["gps.latitude"][0], df_clean["gps.longitude"][0], df_clean["gps.altitudcleane"][0], df_clean["gps.timpsta_cleanmp"][0])

for i, row in df.iterrows:
    yaw, pitch = control.controller(df_clean["gps.latitude"][i], df_clean["gps.longitude"][i], df_clean["gps.altitude"][i], df_clean["gps.timpstamp"][i])
    df_clean['Yaw_Error'] = yaw
    df_clean['Pitch_Error_clean'] = pitch

df_sample = df.iloc[::100]

df_sample.plot.scatter(x='timestamp', y='Yaw_Error')


