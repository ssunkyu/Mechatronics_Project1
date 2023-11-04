import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a pandas DataFrame
file_path = './csv/600.0_0.1_1.0_2.06.csv'  # 파일 경로를 여러분의 파일 경로로 변경하세요.
data = pd.read_csv(file_path, header=None, names=['Time', 'Position'])

# Redefine the final value as the constant 8
final_value = 8.0

# Rise time (time to reach 90% of the final value for the first time)
rise_time = data['Time'][data['Position'] >= 0.9 * final_value].iloc[0]

# Settling time (time after which the position remains within 2% of the final value)
# Find the last time the position was outside the 2% band of the final value
settling_index = data[(data['Position'] < 0.98 * final_value) | (data['Position'] > 1.02 * final_value)].index[-1]
settling_time = data['Time'].iloc[settling_index]

# Overshoot (percentage over the final value)
overshoot_value = data['Position'].max() - final_value
overshoot_percentage = (overshoot_value / final_value) * 100

# Plotting the data with annotations for rise time, settling time, and overshoot
plt.figure(figsize=(12, 6))

# Plot the response
plt.plot(data['Time'], data['Position'], label='Motor Position', color='blue')

# Annotate the rise time
rise_time_position = data['Position'][data['Time'] == rise_time].values[0]
plt.plot([0, rise_time], [rise_time_position, rise_time_position], color='green', linestyle='--')
plt.plot([rise_time, rise_time], [0, rise_time_position], color='green', linestyle='--')
plt.text(rise_time, 0, f'Rise Time: {rise_time:.2f}s', verticalalignment='bottom', horizontalalignment='right', color='green')

# Annotate the settling time
settling_time_position = data['Position'][data['Time'] == settling_time].values[0]
plt.plot([settling_time, settling_time], [0, settling_time_position], color='red', linestyle='--')
plt.text(settling_time, 0, f'Settling Time: {settling_time:.2f}s', verticalalignment='bottom', horizontalalignment='right', color='red')

# Annotate the overshoot
overshoot_time = data['Time'][data['Position'] == data['Position'].max()].values[0]
plt.plot([overshoot_time, overshoot_time], [final_value, data['Position'].max()], color='purple', linestyle='--')
plt.scatter([overshoot_time], [data['Position'].max()], color='purple')
plt.text(overshoot_time, final_value, f'Overshoot: {overshoot_percentage:.2f}%', verticalalignment='bottom', horizontalalignment='right', color='purple')

# Final value line
plt.hlines(final_value, 0, data['Time'].iloc[-1], colors='orange', linestyles='--', label='Final Value (8.0)')

plt.title('Motor Position Response Over Time with Annotations')
plt.xlabel('Time (seconds)')
plt.ylabel('Position')
plt.grid(True)
plt.legend()
plt.show()
