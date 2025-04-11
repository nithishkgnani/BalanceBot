%% Authors: Mufide GULEKEN , Nithish Krishnabharathi GNANI
function K = LQR_Calc_DOF2(K_m,R_m,i_gb,r,m_B,m_W,J,I_2,g,l)
% Parameters
% % Motor/gearbox
% K_m = (6.9401e-04+7.4663e-04)/2;     % Motor constant [Nm/A]
% R_m = (6.1298+6.7397)/2;      % Motor resistance [Ω]
% i_gb = 30;      % Gearbox ratio
% r = 0.04;       % Wheel radius [m]
% 
% % Mass properties
% m_B = 0.368;    % Body mass [kg]
% m_W = 0.02;     % Wheel mass [kg]
% l = 0.01;       % Body CoM offset [m]
% J = 2.175*10^-4;% Wheel inertia [kg·m²]
% g = 9.81;       % Gravity [m/s²]

% Derived parameters
a = m_B * l;
% I_2 = 2.175*10^-4; % Body inertia
I_O = I_2 + m_B*l^2;
m_tot = m_B + 2*m_W;
m_O = m_tot + J/r^2;

% Define symbolic variables
syms x x_dot theta theta_dot U real;

% Motor torque equation
x_w_dot = x_dot / r;
T_m = (K_m/R_m)*U - (K_m^2/R_m)*i_gb*x_w_dot + (K_m^2/R_m)*i_gb*theta_dot;

% Compute d1
d1 = I_O*m_O - (a*cos(theta))^2;

% Nonlinear dynamics (x_ddot and theta_ddot from Equations 21–22)
x_ddot = (1/d1) * (a*I_O*theta_dot^2*sin(theta) - a^2*g*sin(theta)*cos(theta) + T_m*(I_O/r + a*cos(theta)));
theta_ddot = (1/d1) * (-a^2*theta_dot^2*sin(theta)*cos(theta) + a*m_O*g*sin(theta) - T_m*(m_O + (a/r)*cos(theta)));

% State vector derivatives
f = [x_dot; x_ddot; theta_dot; theta_ddot];

% Linearize the System
% Linearize around equilibrium (theta=0, x_dot=0, theta_dot=0)
eq_point = [x, 0, 0, 0]; % theta=0, x_dot=0, theta_dot=0
A = subs(jacobian(f, [x, x_dot, theta, theta_dot]), [x_dot, theta, theta_dot], [0, 0, 0]);
B = subs(jacobian(f, U), [x_dot, theta, theta_dot], [0, 0, 0]);

% Convert symbolic matrices to numeric
A = double(A);
B = double(B);

%% Design LQR Controller
% Define weighting matrices Q and R
Q = diag([1, 0.1, 10, 0.1]); % Prioritize theta (angle) and x (position)
R = 0.01;                     % Penalize control effort

% Compute LQR gain
K = lqr(A, B, Q, R);
end