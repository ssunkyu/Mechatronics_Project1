% Modify filename
filename = '600.0_0.1_1.0.csv';
Ts = 0.01; % Sampling Time
data = readmatrix(filename);
y = data(:, 2);

% PID Value Set
pid_values = regexp(filename, '(\d+\.\d+)', 'tokens');
Kp = str2double(pid_values{1}{1});
Ki = str2double(pid_values{2}{1});
Kd = str2double(pid_values{3}{1});

u = 8 * ones(size(y));  % Step magnitude is 8
data = iddata(y, u, Ts);

% PID controller transfer function
pid_tf = tf([Kd Kp Ki], [1 0]);

% 데이터에서 PID 컨트롤러의 영향 제거
y_openloop = lsim(1/pid_tf, y, linspace(0, Ts*length(y), length(y)));

sys = tfest(u, y_openloop,3,0, 'Ts',Ts);
sys.Numerator
sys.Denominator