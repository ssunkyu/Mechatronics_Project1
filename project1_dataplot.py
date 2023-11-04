import pandas as pd
import matplotlib.pyplot as plt
import re
from scipy.signal import find_peaks
import numpy as np

# Load the CSV file into a pandas DataFrame

file_name = "1200.0_0.1_1.0.csv"
match = re.match(r"(\d+\.\d+)_(\d+\.\d+)_(\d+\.\d+)", file_name)
if match:
    pgain, igain, dgain = match.groups()
    
file_path = './csv/' + file_name
data = pd.read_csv(file_path, header=None, names=['Time', 'redGearPosition'], skipfooter=1, engine='python')

with open(file_path, 'r') as f:
    last_line = f.readlines()[-1].strip()
    _, itae_value_str = last_line.split(',')
    itae_value = float(itae_value_str)


# Define the final value as the constant 8
final_value = 8.0

# Calculate rise time from 10% to 90% of the final value
rise_time_start = data['Time'][data['redGearPosition'] >= 0.1 * final_value].iloc[0]
rise_time_end = data['Time'][data['redGearPosition'] >= 0.9 * final_value].iloc[0]
rise_time = rise_time_end - rise_time_start

# Find peak time
peak_time = data['Time'][data['redGearPosition'] == data['redGearPosition'].max()].iloc[0]

# 최종 값에 대한 허용 오차 범위 설정 (2%)
tolerance = 0.02 * final_value

# 최대값과 최소값(극값) 인덱스 찾기
peaks, _ = find_peaks(data['redGearPosition'], height=final_value * (1 - tolerance))
troughs, _ = find_peaks(-data['redGearPosition'], height=-final_value * (1 + tolerance))

# 모든 극값 인덱스 결합
extrema = np.sort(np.concatenate((peaks, troughs)))

# peak_time 이후 최종 값 범위 내에 들어서는 첫 번째 극값 찾기
for extrema_index in extrema:
    if data['Time'][extrema_index] > peak_time:
        if data['redGearPosition'][extrema_index] <= final_value * (1 + tolerance) and \
           data['redGearPosition'][extrema_index] >= final_value * (1 - tolerance):
            settling_time = data['Time'][extrema_index]
            break


# Calculate steady state error (SSE) as the difference between the final value and the last redGearPosition value
steady_state_error = final_value - data['redGearPosition'].iloc[-1]

# Plotting the updated data with annotations for settling time and SSE
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

# Title and labels
title = f'Motor redGearPosition Response (PGain: {pgain}, IGain: {igain}, DGain: {dgain}, ITAE: {itae_value:.2f})'
plt.title(title)
plt.xlabel('Time (seconds)')
plt.ylabel('redGearPosition')
plt.grid(True)
plt.legend(loc='lower right')
plt.xlim(left=0, right=5.1)
plt.ylim(bottom=0)

# Save the figure
save_file_name = f"{pgain}_{igain}_{dgain}_response.png"
plt.savefig(f'./graph_img/{save_file_name}', bbox_inches='tight')

plt.show()
