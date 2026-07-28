% flightKinematics.m
% Roger Chen
% 2026-06-30
% Modular function to compute the end state of a flight trajectory after
% some time

function [xf, xdotf, zf, zdotf] = flightKinematics(xi, xdoti, zi, zdoti, ts)
    % retrieve constants here
    % g = 9.81;
    % [~, g, ~] = physConstants();
    g = 1;
    xdotf = ts * 0 + xdoti;
    xf = xi + xdoti .* ts;
    zf = zi + (zdoti .* ts) - ((0.5 * g) * ts.^2);
    zdotf = zdoti - (g * ts);
end