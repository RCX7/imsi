% simPopulation.m
% Roger Chen
% 2026-06-15

clc
clear
close all

% Define parameteres
P0 = 50;
alpha = 1;
tfinal = 5;
dt = 0.01;

% Simulate system (solution known)
t_vec = 0:dt:tfinal;
P_vec = P0 * exp(alpha * t_vec);

% Simulate system (approximation, solution not known)
% Euler's method
P_vec_sim = zeros(size(t_vec));     % preallocation
P_vec_sim(1) = P0;                  % set initial value
N = numel(t_vec);
for iter = 2:N                      % compute t_vec.size - 1 times
    prevP = P_vec_sim(iter - 1);
    currP = prevP + dt * (alpha * prevP);
    P_vec_sim(iter) = currP;
end

% Simulate system (approximation, solution not known)
% Runge-Kutta method (ODE45)
dynamics = @(t, x) alpha*x;
[t_ode45, x_ode45] = ode45(dynamics, [0 tfinal], P0);


% Plot the population over time
plot(t_vec, P_vec, 'k-');
xlabel("Time (years)");
ylabel("Population (rabbits)");

hold on;
plot(t_vec, P_vec_sim, 'r-');

plot(t_ode45, x_ode45, "b--");

legend;