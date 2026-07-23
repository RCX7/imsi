% computeForces.m
% Roger Chen
% 2026-06-30
% Computes the force at each time step given the states.
% Assume the point of ground contact is at 0, 0
% Should return a vector of size (1, N)

function forces = computeForces(states, control, mode)

    [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode);
    [xs, xdots, zs, zdots, las] = extractStates(states);

    % dists = sqrt(zs.^2 + xs.^2);
    % ddists = (xs .* xdots + zs .* zdots) ./ dists;
    % forces = (k * (las - dists)) + (c * (control - ddists));

    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);
end