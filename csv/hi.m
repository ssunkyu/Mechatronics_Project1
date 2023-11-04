% Modify filename
filename = '1200.0_0.1_1.0.csv';

na = 3;
nb = 1;
nk = 1;

data = readmatrix(filename);
y = data(:, 2);

% PID Value Set
pid_values = regexp(filename, '(\d+\.\d+)', 'tokens');
Kp = str2double(pid_values{1}{1});
Ki = str2double(pid_values{2}{1});
Kd = str2double(pid_values{3}{1});

u = 8 * ones(size(y));  % Step magnitude is 8
Ts = 0.01; % Sampling Time
data = iddata(y, u, Ts);


% PID controller transfer function
pid_tf = tf([Kd Kp Ki], [1 0]);

% 데이터에서 PID 컨트롤러의 영향 제거
y_openloop = lsim(1/pid_tf, y, linspace(0, Ts*length(y), length(y)));
data_openloop = iddata(y_openloop, u, Ts);

model_arx = arx(data_openloop, [na nb nk]);
plant_tf = tf(model_arx);

C = pid(Kp,Ki,Kd, 'Ts',Ts);
T0 = feedback(plant_tf*C, 1);
[C_opt, info] = pidtune(plant_tf, 'PID');

optimized_Kp = C_opt.Kp;
optimized_Ki = C_opt.Ki;
optimized_Kd = C_opt.Kd;

% 최적화된 PID 컨트롤러를 사용하여 closed-loop 시스템 생성
T_closedloop = feedback(plant_tf*C_opt, 1);

% 주어진 입력 u에 대한 시스템 응답 계산
t = (0:Ts:(length(u)-1)*Ts)';  % 시간 벡터 생성
y_response = lsim(T_closedloop, u, t);

% 응답 플롯
figure;
plot(t, y_response);
xlabel('Time (seconds)');
ylabel('Response y(t)');
title('Closed-loop Response with Optimized PID Controller');
grid on;

% % 입력 데이터 시각화
% time = data_openloop.SamplingInstants;
% u_openloop = data_openloop.u;            
% y_openloop = data_openloop.y;  

% figure;
% subplot(2, 1, 1);
% plot(time, u_openloop);
% title('Open-loop Input (u)');
% xlabel('Time');
% ylabel('Magnitude');
% 
% subplot(2, 1, 2);
% plot(time, y_openloop);
% title('Open-loop Output (y)');
% xlabel('Time');
% ylabel('Magnitude');
% sgtitle('Open-loop Data Visualization');