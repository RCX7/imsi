% constants.m
% a way to access constants globally.

function [g, Lp, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants()
    g = 9.81;                   % Gravity (m/s^2)
    Lp = 0.8;                   % Leg length (meters) - may not be necessary
    m = 0.2;                    % Mass (kg)
    k = 100;                    % Spring constant (N/m)
    c = 0.8;                    % Dampening constant (Ns/m)
    h0 = 1;                     % Starting height (m)
    hdot0 = 0;                 % Starting velocity (m/s)
    N = 60;                     % Number of knot points (coll. points = N - 1)
    n_state_vars = 3;           % Number of state variables
    tfinal = 1.5;                 % Time (seconds)
    dt = (tfinal) / (N - 1);    % time between each
end

