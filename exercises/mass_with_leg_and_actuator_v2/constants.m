% constants.m
% a way to access constants globally.

function [g, Lp, m, k, c, h0, hdot0, tfinal] = constants()
    g = 9.81;            % Gravity (m/s^2)
    Lp = 0.8;            % Leg length (meters) - may not be necessary
    m = 0.2;             % Mass (kg)
    k = 50;              % Spring constant (N/m)
    c = 0.5;             % Dampening constant (Ns/m)
    h0 = 1;                           % Starting height (m)
    hdot0 = -1;                         % Starting velocity (m/s)
    tfinal = 5;                         % Time (seconds)
end

