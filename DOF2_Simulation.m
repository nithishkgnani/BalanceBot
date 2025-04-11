%% Authors: Mufide GULEKEN , Nithish Krishnabharathi GNANI
clear all;
clc
%% MATLAB implementation of Sigi plant model
% ===== Parameters =====
% Motor/gearbox
K_m = (6.9401e-04+7.4663e-04)/2;     % Motor constant [Nm/A]
R_m = (6.1298+6.7397)/2;      % Motor resistance [Ω]
i_gb = 30;      % Gearbox ratio
r = 0.04;       % Wheel radius [m]

% Mass properties
m_B = 0.368;    % Body mass [kg]
m_W = 0.02;     % Wheel mass [kg]
l = 0.01;       % Distance of wheel axle to center of mass [m]
J = 1.92*10^-4;% Wheel inertia [kg·m²]
g = 9.81;       % Gravity [m/s²]
I_2 = 2.175*10^-4; % Body inertia

d_ff = 0.00; % fluid friction coefficient Optional, default 0

%% Simulate the model with state feedback control
% >>>>>>>>  Simulation Setup <<<<<<<<
tspan = [0 15];         % Simulation time [s]
theta_init_deg = 10.0;     % Initial angle

simulate_plant(K_m,R_m,i_gb,r,m_B,m_W,J,I_2,g,d_ff,l,tspan,theta_init_deg);

%% Functions
function simulate_plant(K_m,R_m,i_gb,r,m_B,m_W,J,I_2,g,d_ff,l,tspan,theta_init_deg) 
    % ===== Derived Parameters =====
    a = m_B * l;
    I_O = I_2 + m_B*l^2;
    m_tot = m_B + 2*m_W;
    m_O = m_tot + J/r^2;
    
    % ===== Initial state =====
    x0 = [0; 0; deg2rad(theta_init_deg); 0]; % Initial state: [x, x_dot, theta, theta_dot]
    
    % >>>>>>>> K for state feedback controller <<<<<<<<<
    K = LQR_Calc_DOF2(K_m,R_m,i_gb,r,m_B,m_W,J,I_2,g,l);
    
    % ===== Run ODE Solver =====
    [t, X] = ode45(@(t,x) plant_dynamics(t,x,K_m,R_m,i_gb,r,a,I_O,m_O,g,d_ff,K), tspan, x0);
    
    % ===== Extract Results =====
    x = X(:,1);         % Horizontal position
    x_dot = X(:,2);     % Horizontal velocity
    theta = rad2deg(X(:,3)); % Pitch angle [deg]
    theta_dot = rad2deg(X(:,4)); % Angular velocity
    
    % ===== Plot Results =====
    figure;
    
    subplot(2,2,1);
    plot(t, x, 'b','LineWidth',2.5);
    ylabel({'[m]'});
    legend({'Position $x$ [m]'},'Interpreter','latex');
    title('Linear position');    
    set(gca,'FontSize',14)
    grid on;
    subplot(2,2,2);
    plot(t, x_dot, 'r','LineWidth',2.5);
    ylabel({'[m/s]'});
    xlabel({'Time [s]'});
    legend({'Velocity $\dot{x}$ [m/s]'},'Interpreter','latex');
    title('Linear velocity');    
    set(gca,'FontSize',14)
    grid on;
    
    subplot(2,2,3);
    plot(t, theta, 'g','LineWidth',2.5);
    ylabel({'[deg]'});
    legend({'Pitch $\theta$ [deg]'},'Interpreter','latex');
    title('Angular position');      
    set(gca,'FontSize',14)
    grid on;
    subplot(2,2,4);
    plot(t, theta_dot, 'm','LineWidth',2.5);
    ylabel({'[deg/s]'});
    legend({'Angular Velocity $\dot{\theta}$ [deg/s]'},'Interpreter','latex');
    title('Angular velocity');      
    set(gca,'FontSize',14)
    grid on;    
    
end

function dxdt = plant_dynamics(t,x,K_m,R_m,i_gb,r,a,I_O,m_O,g,d_ff,K)
    % Unpack states
    x_pos = x(1);
    x_dot = x(2);
    theta = x(3);
    theta_dot = x(4);
    
    % ===== Voltage Input (Modify as needed) =====
%     Umax = 6;
%     U = Umax; % Constant 6V input 
%     U = Umax*(t<2); % Step input upto 2s
%     U = Umax*(t >= 10 & t < 30);  % Pulse input (6V from 1s to 3s)
%     U = Umax*sin(2*pi*t);       % Sinusoidal input
    U = -K * [x_pos; x_dot; theta; theta_dot];  % Full state feedback
    
    % ===== Compute Motor Torque T_m =====
    x_w_dot = x_dot/r;       % Wheel angular velocity
    T_m = (K_m/R_m)*U - (K_m^2/R_m)*i_gb*x_w_dot + (K_m^2/R_m)*i_gb*theta_dot;
    
    % ===== Compute d1 =====
    d1 = I_O*m_O - (a*cos(theta))^2;
    
    % ===== Compute Accelerations =====
    if abs(d1) < 1e-6
        error('Singularity in d1 denominator');
    end
    
    % x_double_dot (Equation 21)
    x_ddot = (1/d1)*(...
        a*I_O*theta_dot^2*sin(theta) ...
        - a^2*g*sin(theta)*cos(theta) ...
        + T_m*(I_O/r + a*cos(theta)) );
    
    % Optional fluid friction
    x_ddot = x_ddot - d_ff*x_dot; % Damping term
    
    % theta_double_dot (Equation 22)
    theta_ddot = (1/d1)*(...
        -a^2*theta_dot^2*sin(theta)*cos(theta) ...
        + a*m_O*g*sin(theta) ...
        - T_m*(m_O + (a/r)*cos(theta)) );
    
    % ===== Return Derivatives =====
    dxdt = [x_dot;          % dx/dt = x_dot
            x_ddot;         % d²x/dt² = x_ddot
            theta_dot;      % dθ/dt = theta_dot
            theta_ddot];    % d²θ/dt² = theta_ddot
end