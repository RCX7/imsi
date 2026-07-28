% trajConstraints.m
% Roger Chen
% 2026-06-30
% Generating constraints for fmincon solver.

function [const, const_eq] = trajConstraints(w, target_speed, target_freq, k, c, mode)

    % retrieve constants
    [m, g, la0] = physConstants();
    [const, const_eq] = trajConstraintsBase(w, m, g, la0, target_speed, target_freq, k, c, mode);
end