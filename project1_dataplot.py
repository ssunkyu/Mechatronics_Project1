import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks
import re

# Constants
FINAL_VALUE = 8.0
TOLERANCE = 0.02 * FINAL_VALUE

# Extract the gains from the file name using regular expressions
file_name = "1200.0_0.1_1.0.csv"
match = re.match(r"(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)", file_name)
if match:
    pgain, igain, dgain = match.groups()

# Set file paths
file_path = f'./csv/{file_name}'
graph_img_path = f"./graph_img/{pgain}_{igain}_{dgain}_response.png"

# Read the data and ITAE value from the CSV
data = pd.read_csv(file_path, header=None, names=['Time', 'Position'], skipfooter=1, engine='python')
with open(file_path, 'r') as f:
    itae_value = float(f.readlines()[-1].strip().split(',')[-1])

# Calculate the rise time (10% to 90% of the final value)
rise_time_start = data['Time'][data['Position'] >= 0.1 * FINAL_VALUE].iloc[0]
rise_time_end = data['Time'][data['Position'] >= 0.9 * FINAL_VALUE].iloc[0]
rise_time = rise_time_end - rise_time_start

# Find peak time
peak_time = data['Time'][data['Position'] == data['Position'].max()].iloc[0]

# Find peaks and troughs to determine the settling time
peaks, _ = find_peaks(data['Position'], height=FINAL_VALUE * (1 - TOLERANCE))
troughs, _ = find_peaks(-data['Position'], height=-FINAL_VALUE * (1 + TOLERANCE))
extrema = np.sort(np.concatenate((peaks, troughs)))

# Find the first extremum within the tolerance after the peak time
settling_time = next((data['Time'][extremum] for extremum in extrema 
                      if data['Time'][extremum] > peak_time and 
                      FINAL_VALUE * (1 - TOLERANCE) <= data['Position'][extremum] <= FINAL_VALUE * (1 + TOLERANCE)),
                     None)

# Calculate steady state error (SSE)
steady_state_error = FINAL_VALUE - data['Position'].iloc[-1]

# Plot the motor position response
plt.figure(figsize=(12, 6))
plt.plot(data['Time'], data['Position'], label='Motor Position', color='blue')

# Annotate rise time
plt.annotate(f'10-90% Rise Time: {rise_time:.2f}s', 
             xy=(rise_time_end, FINAL_VALUE * 0.9), 
             xytext=(rise_time_end, FINAL_VALUE * 0.95), 
             arrowprops=dict(facecolor='green', shrink=0.05), 
             color='green')

# Annotate settling time
if settling_time:
    plt.annotate(f'Settling Time: {settling_time:.2f}s', 
                 xy=(settling_time, FINAL_VALUE), 
                 xytext=(settling_time + 0.2, FINAL_VALUE * 0.5), 
                 arrowprops=dict(facecolor='red', shrink=0.05), 
                 color='red')

# Annotate overshoot
overshoot_value = data['Position'].max() - FINAL_VALUE
overshoot_time = data['Time'][data['Position'] == data['Position'].max()].iloc[0]
overshoot_percentage = (overshoot_value / FINAL_VALUE) * 100
plt.annotate(f'Overshoot: {overshoot_percentage:.2f}%', 
             xy=(overshoot_time, data['Position'].max()), 
             xytext=(overshoot_time + 0.1, data['Position'].max() + 0.5), 
             arrowprops=dict(facecolor='purple', shrink=0.05), 
             color='purple')

# Annotate steady state error
plt.annotate(f'SSE: {steady_state_error:.2f}', 
             xy=(data['Time'].iloc[-1], data['Position'].iloc[-1]), 
             xytext=(data['Time'].iloc[-50], data['Position'].iloc[-1] + 0.5), 
             arrowprops=dict(facecolor='orange', shrink=0.05), 
             color='orange')

# Add final value line, title, labels, and legend
plt.axhline(y=FINAL_VALUE, color='orange', linestyle='--', label='Final Value')
plt.title(f'Motor Position Response (PGain: {pgain}, IGain: {igain}, DGain: {dgain}, ITAE: {itae_value:.2f})')
plt.xlabel('Time (seconds)')
plt.ylabel('Position')
plt.grid(True)
plt.legend(loc='lower right')
plt.xlim(left=0)
plt.ylim(bottom=0)

# Save the figure and display the plot
plt.savefig(graph_img_path, bbox_inches='tight')
plt.show()
