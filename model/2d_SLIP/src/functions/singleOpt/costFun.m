% costFun.m
% Roger Chen
% 2026-06-30
% A cost function to minimize energy expenditure for trajectory
% optimization

function [cost] = costFun(w, mode)

    [u, states, addDecs] = decToMats(w);
    [N, n_states] = simConstants();
    [xs, xdots, zs, zdots, las] = extractStates(states);
    [xf, xdotf, zf, zdotf] = flightKinematics( ...
        xs(end), xdots(end), zs(end), zdots(end), addDecs(2));
    
    % swingCost = 0;
    dist = xs(end) - xs(1);
    dtravel = xf - xs(1);
    % input 'r' for running, 'h' for hopping.
    swingCost = computeSwingWork(dist, addDecs(1), addDecs(2), mode, dtravel);
    cost = trapz(addDecs(1), addDecs(3:2+N)) + swingCost;
end