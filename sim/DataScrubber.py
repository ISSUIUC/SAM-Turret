import pandas as pd
from sys import argv
# takes single command line argument of file name

df = pd.read_csv(argv[1])
df = df.dropna(subset=['gps.latitude'])
df = df[df['gps.latitude'] != 0]
print('Good')
df_clean = df[['gps.latitude', "gps.longitude", "gps.altitude", "timestamp"]]
df_clean.to_csv(argv[1] + '_clean.csv', index=False)