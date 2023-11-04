% Read the CSV file into a table
T = readtable('800.00_0.10_100.00.csv', 'ReadVariableNames', false);

% Convert the table to an array
Array = table2array(T(1:end-1, :)); % Exclude the last row with ITAE

% Assign the columns to variables
t = Array(:, 1);
y = Array(:, 2);

% Define the final value
y_final_value = 8;

% Calculate step info
info = stepinfo(y, t, y_final_value);

disp(info)

% Plot the response
plot(t, y);
title('Step Response');
xlabel('Time (seconds)');
ylabel('Output');
