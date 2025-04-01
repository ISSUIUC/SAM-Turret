import pandas as pd
import matplotlib as mpl
import Sammy
import Motor
import yaml

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)  # safe_load is used to prevent arbitrary code execution
    return data

file_path = 'turret_config.yaml'  # Replace with your YAML file path
parsed_data = load_yaml(file_path)

motor_1_specifications = parsed_data['motor_1_specifications']
step_size_1 = motor_1_specifications['step_size']
actuation_time_1 = motor_1_specifications['actuation_time']
dead_zone_1 = motor_1_specifications['dead_zone']

motor_1 = Motor.Motor(step_size_1, actuation_time_1, dead_zone_1) # pitch

motor_2_specifications = parsed_data['motor_2_specifications']
step_size_2 = motor_2_specifications['step_size']
actuation_time_2 = motor_2_specifications['actuation_time']
dead_zone_2 = motor_2_specifications['dead_zone']

motor_2 = Motor.Motor(step_size_2, actuation_time_2, dead_zone_2) # yaw

K_list_1 = parsed_data['K_list_1']
Kp_1 = K_list_1['Kp']
Ki_1 = K_list_1['Ki']
Kd_1 = K_list_1['Kd']

controller_1 = Sammy.PID(Kp_1, Ki_1, Kd_1)

K_list_2 = parsed_data['K_list_2']
Kp_2 = K_list_2['Kp']
Ki_2 = K_list_2['Ki']
Kd_2 = K_list_2['Kd']

controller_2 = Sammy.PID(Kp_2, Ki_2, Kd_2)

turret_position = parsed_data['turret_position']
turret_latitude = turret_position['latitude']
turret_longitude = turret_position['longitude']
turret_altitude = turret_position['altitude']
pos_turr = [turret_latitude, turret_longitude, turret_altitude]

rocket_position = parsed_data['rocket_position']
rocket_latitude = rocket_position['latitude']
rocket_longitude = rocket_position['longitude']
rocket_altitude = rocket_position['altitude']
pos_rocket = [turret_latitude, turret_longitude, turret_altitude]

Sammy = Sammy.Sammy(pos_turr, pos_rocket, motor_1, motor_2, controller_1, controller_2)

df_clean = pd.read_csv("data_clean.csv")   # read from cleaned (smaller) file
# print(df_clean.dtypes)

# df_lat = df[df["gps.latitude"]]
# df_lon = df[df["gps.longitude"]]
# df_alt = df[df["gps.altitude"]]
# df_time = df[df["timestamp"]]

df_output = pd.DataFrame(columns = ['pitch_actual', 'yaw_actual', 
                                    'pitch_desired', 'yaw_desired', 
                                    'motor_1_step_increment', 'motor_2_step_increment'])

for i in range(len(df_clean)): 
    row = df_clean.loc[i]
    lat = row['gps.latitude']
    long = row['gps.longitude']
    alt = row['gps.altitude']
    time = row['timestamp']
    [[pitch_actual, yaw_actual], 
     [pitch_desired, yaw_desired], 
     [motor_1_step_increment, motor_2_step_increment]
     ] = Sammy.update([lat, long, alt], time)
    new_row = {'pitch_actual' : pitch_actual, 'yaw_actual' : yaw_actual, 
               'pitch_desired' : pitch_desired, 'yaw_desired' : yaw_desired, 
               'motor_1_step_increment' : motor_1_step_increment, 'motor_2_step_increment' : motor_2_step_increment}
    df_output.loc[i] = new_row

df_output.to_csv('output.csv', index=False)

'''

control = Controller(turret_lat, turret_lon, 0, df_clean["gps.latitude"][0], df_clean["gps.longitude"][0], df_clean["gps.altitudcleane"][0], df_clean["gps.timpsta_cleanmp"][0])

for i, row in df.iterrows:
    yaw, pitch = control.controller(df_clean["gps.latitude"][i], df_clean["gps.longitude"][i], df_clean["gps.altitude"][i], df_clean["gps.timpstamp"][i])
    df_clean['Yaw_Error'] = yaw
    df_clean['Pitch_Error_clean'] = pitch

df_sample = df.iloc[::100]

df_sample.plot.scatter(x='timestamp', y='Yaw_Error')


'''