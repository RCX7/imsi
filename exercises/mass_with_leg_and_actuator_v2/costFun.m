% costFun.m
% Roger Chen
% 2026-18-06
% Attempts to optimize to get mass to always bounce up to the same height

function [J] = costFun(w)
    % 1st cost function: mse from desired height
    % [g, la0, m, k, c, h0, hdot0, tfinal] = constants();

    % desiredHeight = 1;                      % try to always hop to one meter
    % [peaks, las, hfinal] = simulateHop(w);
    % J = sum((peaks - desiredHeight).^2);    % simple mse

    % 2nd cost function: minimize activations
    %J = sum(cumtrapz(w));

    % Random third cost function
    %J = -(w(end));
    J = 0; % for debugging
end