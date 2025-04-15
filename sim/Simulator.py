import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import pandas as pd
import matplotlib.pyplot as plt
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

motor_1 = Motor.Motor(step_size_1, actuation_time_1) # pitch

motor_2_specifications = parsed_data['motor_2_specifications']
step_size_2 = motor_2_specifications['step_size']
actuation_time_2 = motor_2_specifications['actuation_time']

motor_2 = Motor.Motor(step_size_2, actuation_time_2) # yaw

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
pos_rocket = [rocket_latitude, rocket_longitude, rocket_altitude]

Sammy = Sammy.Sammy(pos_turr, pos_rocket, motor_1, motor_2, controller_1, controller_2)

df_clean = pd.read_csv("data_clean.csv")   # read from cleaned (smaller) file

df_output = pd.DataFrame(columns = ['pitch_actual', 'yaw_actual', 
                                    'pitch_desired', 'yaw_desired', 
                                    'motor_1_step_increment', 'motor_2_step_increment'])

for i in range(len(df_clean)): 
    row = df_clean.loc[i]
    time = row['timestamp']
    lat = row['gps.latitude']
    long = row['gps.longitude']
    alt = row['gps.altitude']
    [[pitch_actual, yaw_actual], 
     [pitch_desired, yaw_desired], 
     [motor_1_step_increment, motor_2_step_increment]
     ] = Sammy.update([lat, long, alt], time)
    new_row = {'pitch_actual' : pitch_actual, 'yaw_actual' : yaw_actual, 
               'pitch_desired' : pitch_desired, 'yaw_desired' : yaw_desired, 
               'motor_1_step_increment' : motor_1_step_increment, 'motor_2_step_increment' : motor_2_step_increment}
    df_output.loc[i] = new_row

df_output.to_csv('output.csv', index=False)

df_data = pd.read_csv("data.csv")

# Plot launch altitude
plt.figure(figsize=(10,4))
plt.plot(df_clean['timestamp'], df_clean['gps.altitude'], label='Altitude')
plt.title('Altitude vs Timestamp')
plt.xlabel('Timestamp')
plt.ylabel('Altitude (m)')
plt.grid(True)
plt.legend()
plt.tight_layout()

# Plot pitch_actual and pitch_desired
plt.figure(figsize=(10, 4))
plt.plot(df_output['pitch_actual'], label='Pitch Actual')
plt.plot(df_output['pitch_desired'], label='Pitch Desired')
plt.title('Pitch: Actual vs Desired')
plt.xlabel('Index')
plt.ylabel('Pitch (deg)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Plot yaw_actual and yaw_desired
plt.figure(figsize=(10, 4))
plt.plot(df_output['yaw_actual'], label='Yaw Actual')
plt.plot(df_output['yaw_desired'], label='Yaw Desired')
plt.title('Yaw: Actual vs Desired')
plt.xlabel('Index')
plt.ylabel('Yaw (deg)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()


'''

control = Controller(turret_lat, turret_lon, 0, df_lat[0], df_lon[0], df_alt[0], df_time[0])

for i, row in df.iterrows:
    yaw, pitch = control.controller(df_lat[i], df_lon[i], df_alt[i], df_time[i])
    df['Yaw_Error'] = yaw
    df['Pitch_Error'] = pitch

df_sample = df.iloc[::100]

df_sample.plot.scatter(x='timestamp', y='Yaw_Error')


'''
