import pandas as pd
from sys import argv
# takes single command line argument of file name

df = pd.read_csv(argv[1])

df = df.dropna(subset=['gps.latitude'])
df = df[df['gps.latitude'] != 0]

df_clean = df[['timestamp', 'gps.latitude', 'gps.longitude', 'gps.altitude']]
df_clean.loc[:, 'gps.latitude'] = df_clean['gps.latitude'] / 1e7
df_clean.loc[:, 'gps.longitude'] = df_clean['gps.longitude'] / 1e7

# sort by timestamp so always increasing
df_clean = df_clean.sort_values(by='timestamp')

# use data starting at the peak and until a timestamp of 400000
max_alt_idx = df_clean['gps.altitude'].idxmax()
df_clean = df_clean.loc[max_alt_idx:]
df_clean = df_clean[df_clean['timestamp'] <= 400000]

# save result
df_clean.to_csv(argv[1][:-4] + '_clean.csv', index=False)