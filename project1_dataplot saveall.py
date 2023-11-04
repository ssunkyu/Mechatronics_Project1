import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks
import re

# Constants
final_value = 8.0   
TOLERANCE = 0.02 * final_value

# Get a list of all CSV files in the `./csv` directory
csv_files = [f for f in os.listdir('./csv') if f.endswith('.csv')]

# Extract the gains from the file name using regular expressions
for file_name in csv_files:
    match = re.match(r"(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)", file_name)
    if match:
        pgain, igain, dgain = match.groups()

    # Set file paths
    file_path = f'./csv/{file_name}'
    graph_img_path = f"./graph_img/{pgain}_{igain}_{dgain}_response.png"

    # Read the data and ITAE value from the CSV
    data = pd.read_csv(file_path, header=None, names=['Time', 'redGearPosition'], skipfooter=1, engine='python')
    with open(file_path, 'r') as f:
        itae_value = float(f.readlines()[-1].strip().split(',')[-1])

    # Calculate the rise time (10% to 90% of the final value)
    rise_time_start = data['Time'][data['redGearPosition'] >= 0.1 * final_value].iloc[0]
    rise_time_end = data['Time'][data['redGearPosition'] >= 0.9 * final_value].iloc[0]
    rise_time = rise_time_end - rise_time_start

    # Find peak time
    peak_time = data['Time'][data['redGearPosition'] == data['redGearPosition'].max()].iloc[0]

    # Find peaks and troughs to determine the settling time
    peaks, _ = find_peaks(data['redGearPosition'], height=final_value * (1 - TOLERANCE))
    troughs, _ = find_peaks(-data['redGearPosition'], height=-final_value * (1 + TOLERANCE))
    extrema = np.sort(np.concatenate((peaks, troughs)))

    # Find the first extremum within the tolerance after the peak time
    settling_time = next((data['Time'][extremum] for extremum in extrema 
                        if data['Time'][extremum] > peak_time and 
                        final_value * (1 - TOLERANCE) <= data['redGearPosition'][extremum] <= final_value * (1 + TOLERANCE)),
                        None)

    # Calculate steady state error (SSE)
    steady_state_error = final_value - data['redGearPosition'].iloc[-1]

    # Plot the motor redGearPosition response
    plt.figure(figsize=(12, 6))
    plt.plot(data['Time'], data['redGearPosition'], label='Motor redGearPosition', color='blue')

    # Annotate the rise time (10% to 90% of the final value)
    plt.plot([0, rise_time_end], [final_value * 0.9, final_value * 0.9], color='green', linestyle='--')
    plt.plot([rise_time_start, rise_time_start], [0, final_value * 0.1], color='green', linestyle='--')
    plt.plot([rise_time_end, rise_time_end], [0, final_value * 0.9], color='green', linestyle='--')
    plt.text(rise_time_end, final_value * 0.9, f'10-90% Rise Time: {rise_time:.2f}s', color='green')

    # Annotate the settling time (first time after peak within 2% of final value)
    plt.plot([settling_time, settling_time], [0, final_value], color='red', linestyle='--')
    plt.text(settling_time, final_value * 0.5, f'Settling Time: {settling_time:.2f}s', color='red')

    # Annotate the overshoot (percentage over the final value)
    # Calculate overshoot as the value over the final value
    overshoot_value = data['redGearPosition'].max() - final_value
    overshoot_time = data['Time'][data['redGearPosition'] == data['redGearPosition'].max()].iloc[0]
    overshoot_percentage = (data['redGearPosition'].max() - final_value) / final_value * 100
    plt.plot([overshoot_time, overshoot_time], [final_value, data['redGearPosition'].max()], color='purple', linestyle='--')
    plt.scatter([overshoot_time], [data['redGearPosition'].max()], color='purple')
    plt.text(overshoot_time, final_value + overshoot_value, f'Overshoot: {overshoot_percentage:.2f}%', color='purple')

    # Annotate steady state error
    plt.text(data['Time'].iloc[-200], data['redGearPosition'].iloc[-1]+0.1, f'Steady state error: {steady_state_error:.2f}', color='orange')
    # Final value line
    plt.hlines(final_value, 0, data['Time'].iloc[-1], colors='orange', linestyles='--', label='Final Value (8.0)')

    # Add final value line, title, labels, and legend
    plt.axhline(y=final_value, color='orange', linestyle='--', label='Final Value')
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
