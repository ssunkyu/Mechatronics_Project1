import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  
import re
import control

# Constants
input_value = 10.0   
TOLERANCE = 0.02

# Get a list of all CSV files in the `./csv` directory
csv_files = [f for f in os.listdir('./csv/DGAIN') if f.endswith('.csv')]

# Extract the gains from the file name using regular expressions
for file_name in csv_files:
    match = re.match(r"(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)", file_name)
    if match:
        pgain, igain, dgain = match.groups()
    
    print(file_name)

    # Set file paths
    file_path = f'./csv/DGAIN/{file_name}'
    graph_img_path = f"./graph_img/DGAIN/{pgain}_{igain}_{dgain}_response.png"

    # Read the data and ITAE value from the CSV
    data = pd.read_csv(file_path, header=None, names=['Time', 'redGearPosition'], skipfooter=1, engine='python')
    with open(file_path, 'r') as f:
        itae_value = float(f.readlines()[-1].strip().split(',')[-1])
        
    steady_state_value = data['redGearPosition'].iloc[-1]
    
    S = control.step_info(data['redGearPosition'], T=data['Time'], T_num=None, yfinal=input_value, SettlingTimeThreshold=0.02, RiseTimeLimits=(0.1, 0.9))
    
    rise_time = S['RiseTime']
    peak_time = S['PeakTime']
    settling_time = S['SettlingTime']
    steady_state_error = S['SteadyStateValue'] - input_value
    
    # Calculate the rise time (10% to 90% of the final value)
    rise_time_start = data['Time'][data['redGearPosition'] >= 0.1 * input_value].iloc[0]
    rise_time_end = data['Time'][data['redGearPosition'] >= 0.9 * input_value].iloc[0]

    # Plot the motor redGearPosition response
    plt.figure(figsize=(12, 6))
    plt.plot(data['Time'], data['redGearPosition'], label='Motor redGearPosition', color='blue')

    # Annotate the rise time (10% to 90% of the final value)
    plt.plot([0, rise_time_end], [input_value * 0.9, input_value * 0.9], color='green', linestyle='--')
    plt.plot([rise_time_start, rise_time_start], [0, input_value * 0.1], color='green', linestyle='--')
    plt.plot([rise_time_end, rise_time_end], [0, input_value * 0.9], color='green', linestyle='--')
    plt.text(rise_time_end, input_value * 0.9, f'10-90% Rise Time: {rise_time:.2f}s', color='green')

    # Annotate the settling time (first time after peak within 2% of final value)
    plt.plot([settling_time, settling_time], [0, input_value], color='red', linestyle='--')
    plt.text(settling_time, input_value * 0.5, f'Settling Time: {settling_time:.2f}s', color='red')

    # Annotate the overshoot (percentage over the final value)
    # Calculate overshoot as the value over the final value
    overshoot_value = data['redGearPosition'].max() - input_value
    overshoot_time = data['Time'][data['redGearPosition'] == data['redGearPosition'].max()].iloc[0]
    overshoot_percentage = (data['redGearPosition'].max() - input_value) / input_value * 100
    plt.plot([overshoot_time, overshoot_time], [input_value, data['redGearPosition'].max()], color='purple', linestyle='--')
    plt.scatter([overshoot_time], [data['redGearPosition'].max()], color='purple')
    plt.text(overshoot_time, input_value + overshoot_value, f'Overshoot: {overshoot_percentage:.2f}%', color='purple')

    # Annotate steady state error
    plt.text(data['Time'].iloc[-200], steady_state_value+0.1, f'Steady state error: {steady_state_error:.2f}', color='orange')
    # Final value line
    plt.hlines(input_value, 0, data['Time'].iloc[-1], colors='orange', linestyles='--', label='Final Value (8.0)')

    # Add final value line, title, labels, and legend
    plt.axhline(y=input_value, color='orange', linestyle='--', label='Final Value')
    plt.title(f'Motor redGearPosition Response (PGain: {pgain}, IGain: {igain}, DGain: {dgain}, ITAE: {itae_value:.2f})')
    plt.xlabel('Time (seconds)')
    plt.ylabel('redGearPosition')
    plt.grid(True)
    plt.legend(loc='lower right')
    plt.xlim(left=0)
    plt.ylim(bottom=0)

    # Save the figure and display the plot
    plt.savefig(graph_img_path, bbox_inches='tight')
    
    # plt.show()
