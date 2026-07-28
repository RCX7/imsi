% computeSlopes.m
% Roger Chen
% 2026-06-30
% Given the states of a SLIP model (x, xdot, z, dot), computes the slopes
% to enforce the kinematics for direct collocation.

% Inputs: states of size (n_states, N)
% current expectation of rows is (x, xdot, z, zdot, la).
% computes slopes with trapezoidal rule
% Returns a matrix of size (n_states, N-1)

function slopes = computeSlopes(states, control, k, c)

    [m, g, ~] = physConstants();
    [xs, xdots, zs, zdots, las] = extractStates(states);
    slopes = computeSlopesBase(xs, xdots, zs, zdots, las, control, m, g, k, c);
end
