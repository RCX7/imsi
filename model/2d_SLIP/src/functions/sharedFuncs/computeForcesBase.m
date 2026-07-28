% computeForcesBase.m
% Roger Chen
% 2026-07-21
% Computes forces given input values
% A base function to modularize the logic for computing force. 
% Called by single-opt and multi-opt compute forces.

function forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c)

    dists = sqrt(zs.^2 + xs.^2);
    ddists = (xs .* xdots + zs .* zdots) ./ dists;
    forces = (k * (las - dists)) + (c * (control - ddists));
end