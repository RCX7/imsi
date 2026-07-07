% simPendulum
% Roger Chen
% 2026-06-15

clc
clear
close all

% Define values
m = 0.8;     % 0.8 kg mass
L = 1;       % 1 meter
g = 9.81;    % gravity

tfinal = 20; % 10 seconds
theta0 = 15;
w0 = 0;

% Simulate system (using ode45)
% define dynamics variable
dynamics = @(t, y) [y(2); (-(g / L) * sind(y(1)))];
[t_vec, theta_vec] = ode45(dynamics, [0 tfinal], [theta0; w0]);

subplot(2, 1, 1);
plot(t_vec, theta_vec(:,1), 'r-');
ylabel("angular displacement (deg)");
xlabel("time (sec)");

subplot(2, 1, 2);
plot(t_vec, theta_vec(:,2), 'b--');
ylabel("angular velocity (deg/sec)");
xlabel("time (sec)");