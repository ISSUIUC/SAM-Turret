import os
import sys
import yaml
import pandas as pd
import matplotlib.pyplot as plt

# Add parent directory to sys.path for module imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from motor import Motor
from sammy import Sammy
from pid import PID

def load_yaml_config(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def initialize_motor(motor_config):
    return Motor(motor_config['step_size'], motor_config['gear_ratio'], motor_config['actuation_time'])

def initialize_pid_controller(K_list):
    return PID([K_list['Kp'], K_list['Ki'], K_list['Kd']])

def process_tracking_data(turret_ctrl, df_clean):
    output = pd.DataFrame(columns=[
        'pitch_actual', 'yaw_actual',
        'pitch_desired', 'yaw_desired',
        'motor_1_step_increment', 'motor_2_step_increment'
    ])

    for i, row in df_clean.iterrows():
        time = row['timestamp']
        lat = row['gps.latitude']
        lon = row['gps.longitude']
        alt = row['gps.altitude']

        (
            [pitch_actual, yaw_actual],
            [pitch_desired, yaw_desired],
            [motor_1_step_inc, motor_2_step_inc]
        ) = turret_ctrl.update([lat, lon, alt], time)

        output.loc[i] = {
            'pitch_actual': pitch_actual,
            'yaw_actual': yaw_actual,
            'pitch_desired': pitch_desired,
            'yaw_desired': yaw_desired,
            'motor_1_step_increment': motor_1_step_inc,
            'motor_2_step_increment': motor_2_step_inc
        }

    return output

def plot_results(df_clean, df_output):
    plt.figure(figsize=(10, 4))
    plt.plot(df_clean['timestamp'], df_clean['gps.altitude'], label='Altitude')
    plt.title('Altitude vs Timestamp')
    plt.xlabel('Timestamp')
    plt.ylabel('Altitude (m)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.figure(figsize=(10, 4))
    plt.plot(df_output['pitch_actual'], label='Pitch Actual')
    plt.plot(df_output['pitch_desired'], label='Pitch Desired')
    plt.title('Pitch: Actual vs Desired')
    plt.xlabel('Index')
    plt.ylabel('Pitch (deg)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

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

def main():
    config = load_yaml_config(r"config/turret_config.yaml")
    if not config:
        print("Error: Configuration file is empty or not found.")
        return
    print("Configuration loaded successfully.")
    print("Initializing motors and controllers...")

    motor_1 = initialize_motor(config['motor_specs_pitch'])
    motor_2 = initialize_motor(config['motor_specs_yaw'])

    controller_1 = initialize_pid_controller(config['K_list_pitch'])
    controller_2 = initialize_pid_controller(config['K_list_yaw'])

    turret_pos = config['turret_position']
    rocket_pos = config['rocket_position']

    pos_turret = [turret_pos['latitude'], turret_pos['longitude'], turret_pos['altitude']]
    pos_rocket = [rocket_pos['latitude'], rocket_pos['longitude'], rocket_pos['altitude']]


    turret_ctrl = Sammy(pos_turret, pos_rocket, motor_1, motor_2, controller_1, controller_2)

    df_clean = pd.read_csv(r"data/data_clean.csv")
    
    print("Data loaded successfully. Processing tracking data...")
    df_output = process_tracking_data(turret_ctrl, df_clean)

    output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'output')
    os.makedirs(output_dir, exist_ok=True)
    output_file_path = os.path.join(output_dir, 'output.csv')
    df_output.to_csv(output_file_path, index=False)

    print("Tracking data processed and saved to output.csv.")
    plot_results(df_clean, df_output)

if __name__ == "__main__":
    main()
