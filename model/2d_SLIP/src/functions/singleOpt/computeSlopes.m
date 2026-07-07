% computeSlopes.m
% Roger Chen
% 2026-06-30
% Given the states of a SLIP model (x, xdot, z, dot), computes the slopes
% to enforce the kinematics for direct collocation.

% Inputs: states of size (n_states, N)
% current expectation of rows is (x, xdot, z, zdot, la).
% computes slopes with trapezoidal rule
% Returns a matrix of size (n_states, N-1)

function slopes = computeSlopes(states, control, mode)

    [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode);
    [N, n_states] = simConstants();
    slopes = zeros(n_states, N-1);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForces(states, control, mode);

    slopes(1,:) = 0.5 * (xdots(1:end-1) + xdots(2:end));
    slopes(2,:) = 0.5 * ( ...
                   ((xs(1:end-1)./ las(1:end-1)) .* forces(1:end-1)) +...
                   ((xs(2:end)./ las(2:end)) .* forces(2:end)) ...
                   ) / m;
    slopes(3,:) = 0.5 * (zdots(1:end-1) + zdots(2:end));
    slopes(4,:) = 0.5 * ( ...
                   ( ...
                   ((zs(1:end-1)./ las(1:end-1)) .* forces(1:end-1)) +...
                   ((zs(2:end)./ las(2:end)) .* forces(2:end)) ...
                   ) / m ...
                 ) - g;
    slopes(5,:) = 0.5 * (control(1:end-1) + control(2:end));
end
