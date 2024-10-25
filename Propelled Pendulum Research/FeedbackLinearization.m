% Feedback Linearization with Reference Tracking Simulation
% Author: [Your Name]
% Date: [Current Date]

% Clear workspace and command window
clear all; close all; clc;

%% System Parameters
m = 1.8667;     % Mass (from original equation coefficient)
b = 0.50;       % Damping coefficient
k = 7.8480;     % Stiffness coefficient

%% Control Gains (Pole Placement)
lambda = 2;         % Desired pole location
K1 = lambda^2;      % Gain for error in position
K2 = 2*lambda;      % Gain for error in velocity

%% Reference Input
theta_ref = 0.8;    % Reference angle in radians

%% Simulation Time
t_start = 0;
t_end = 10;     % Simulate for 10 seconds
tspan = [t_start, t_end];

%% Initial Conditions
x0 = [0; 0];    % Initial state [theta(0); theta_dot(0)]

%% Solve the Differential Equations
% Use ODE45 solver
[t, x] = ode45(@(t, x) dynamics(t, x, m, b, k, K1, K2, theta_ref), tspan, x0);

%% Plotting the Results
% Theta (x1) vs Time
figure;
plot(t, x(:,1), 'LineWidth', 2);
hold on;
yline(theta_ref, 'r--', 'LineWidth', 2); % Reference line
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Angular Position \theta vs Time');
legend('\theta(t)', '\theta_{ref}');
grid on;

% Theta_dot (x2) vs Time
figure;
plot(t, x(:,2), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta_{dot} (rad/s)');
title('Angular Velocity \theta_{dot} vs Time');
grid on;

%% Control Input u vs Time
% Compute control input u over time
u = zeros(length(t), 1);
for i = 1:length(t)
    u(i) = control_law(t(i), x(i,:)', m, b, k, K1, K2, theta_ref);
end

figure;
plot(t, u, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Control Input u');
title('Control Input u vs Time');
grid on;

%% Dynamics Function Definition
function dxdt = dynamics(t, x, m, b, k, K1, K2, theta_ref)
    % Extract state variables
    x1 = x(1);  % theta
    x2 = x(2);  % theta_dot

    % Compute control input u
    u = control_law(t, x, m, b, k, K1, K2, theta_ref);

    % State derivatives
    dx1dt = x2;
    dx2dt = (2*u - b*x2 - k*sin(x1)) / m;

    % Return state derivatives
    dxdt = [dx1dt; dx2dt];
end

%% Control Law Function Definition
function u = control_law(t, x, m, b, k, K1, K2, theta_ref)
    % Extract state variables
    x1 = x(1);  % theta
    x2 = x(2);  % theta_dot

    % Error terms
    e1 = x1 - theta_ref;
    e2 = x2;

    % Nonlinear functions
    f_x = (-b * x2 - k * sin(x1)) / m;

    % Desired error dynamics
    e2_dot_desired = -K1 * e1 - K2 * e2;

    % Compute control input u
    u = (m / 2) * (e2_dot_desired - f_x);

    % Alternatively, using the derived control law:
    % u = -0.9333 * K1 * (x1 - theta_ref) + (-0.9333 * K2 + 0.25) * x2 + 3.9240 * sin(x1);
end
