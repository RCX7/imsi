% computeForces.m
% Roger Chen
% 2026-06-30
% Computes the force at each time step given the states.
% Assume the point of ground contact is at 0, 0
% Should return a vector of size (1, N)

function forces = computeForces(states, control, mode)

    [~, ~, k, c, ~, ~, ~, ~, ~, ~, ~] = physConstants(mode);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);
end