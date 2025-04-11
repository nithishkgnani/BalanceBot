%% Authors: Mufide GULEKEN , Nithish Krishnabharathi GNANI
% clear all;
clc;
%% Simulation parameters for Sigi
T_sim = 15;

% Plant
J = 2.175 * 10^-4; % kg * m^2 % moi of pendulum
m = 0.368; % mass of the body
g = 9.81; % gravity
l = 0.01; % Length 0.01
% J = m*l^2;

d = 0.001; % fluid friction
k_m = (6.9401e-04+7.4663e-04)/2; % motor constant 0.0017 0.0014
R = (6.1298+6.7397)/2; % resistance of motor coil 9.4
K = k_m; % 2 motors
igb = 30; % gear box ratio
k=K;

a = 5.82;
% tau = (1/(a*(a^0.5)*(K/J)))^0.5;
tau = 0.016766357422118;

umax = 5.0; %V
yref = 0.00;
init_theta = 1.00*pi/180;

sat_percentage = 0.8;

% Manual Tuning
Kp = 10  + 2  ;
Ki = 0   + 0.5  ;
Kd = 1.06+ 0;

% % model reference following
% Kp = +45.025;
% Ki = 1.6125e+03;
% Kd = 1.0045;

% Reference model following approach
tau_m = 0.1;
% Kp = -(2*K^2*igb + d*R)/(tau_m*2*K); % two motors => K -> 2*K
% Ki = 0*m*g*l*R/(tau_m*2*K); % two motors => K, K^2 -> 2*K, 2*K^2
% Kd = J*R/(tau_m*2*K); % two motors => K -> 2*K

% feedback linearization
% Kp = (2*K^2*igb)/(tau_m*2*K);
% Kd = J*R/(tau_m*2*K);

% Poles
% s_1 = (K^2-R*d + ((K^2-R*d)^2-4*R^2*J*m*g*l)^(1/2))/(2*R*J);
% s_2 = (K^2-R*d - ((K^2-R*d)^2-4*R^2*J*m*g*l)^(1/2))/(2*R*J);

yrefState = [0 0];
KVec = [Kp Kd];

%% Plotting Lead Phase

x_data = out.logsout{1}.Values.Time;
y_data_ref = rad2deg(out.logsout{1}.Values.Data);
y_data = rad2deg(out.logsout{2}.Values.Data);

figure;
plot(x_data, y_data_ref, 'LineWidth',2.5);
hold on;
plot(x_data, y_data, 'LineWidth',2.5);
ylabel({'[degree]'});
legend({'Reference angle','Pitch angle'});
set(gca,'FontSize',14)
grid on;