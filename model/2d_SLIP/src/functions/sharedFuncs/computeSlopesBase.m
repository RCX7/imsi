% computeSlopesBase.m
% Roger Chen
% 2026-07-24
% Base function to modularize the logic for the single and multi opt
% versions

function slopes = computeSlopesBase(xs, xdots, zs, zdots, las, control, m, g, k, c)

    [N, n_states] = simConstants();
    slopes = zeros(n_states, N-1);
    dists = sqrt(xs.^2 + zs.^2);
    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);

    slopes(1,:) = 0.5 * (xdots(1:end-1) + xdots(2:end));
    slopes(2,:) = 0.5 * ( ...
                   ((xs(1:end-1)./ dists(1:end-1)) .* forces(1:end-1)) +...
                   ((xs(2:end)./ dists(2:end)) .* forces(2:end)) ...
                   ) / m;
    slopes(3,:) = 0.5 * (zdots(1:end-1) + zdots(2:end));
    slopes(4,:) = 0.5 * ( ...
                   ( ...
                   ((zs(1:end-1)./ dists(1:end-1)) .* forces(1:end-1)) +...
                   ((zs(2:end)./ dists(2:end)) .* forces(2:end)) ...
                   ) / m ...
                 ) - g;
    slopes(5,:) = 0.5 * (control(1:end-1) + control(2:end));
end