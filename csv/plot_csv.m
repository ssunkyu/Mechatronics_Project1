Array = csvread('200.0_0.1_1.0_2.10.csv');
t = Array(:, 1);
y = Array(:, 2);
y_final_value = 8;
y_final = y_final_value*ones(size(y));
stepinfo(y, t, y_final_value)



plot(t,y)