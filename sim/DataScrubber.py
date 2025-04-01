import pandas as pd
from sys import argv
# takes single command line argument of file name

df = pd.read_csv(argv[1])

df = df.dropna(subset=['gps.latitude'])
df = df[df['gps.latitude'] != 0]

df_clean = df[['gps.latitude', "gps.longitude", "gps.altitude", "timestamp"]]
df_clean['gps.latitude'] = df_clean['gps.latitude'] / 1e7
df_clean['gps.longitude'] = df_clean['gps.longitude'] / 1e7

df_clean.to_csv(argv[1][:-4] + '_clean.csv', index=False)