% costFunBase.m
% Roger Chen
% 2026-07-23
% A base cost function to modularize logic for computing cost. Like computeForcesBase and
% computeSlopesBase, single and multi opt versions call this function.

function [cost, stanceCost, swingCost, tstance, tflight] = costFunBase(w, m, g, I, mode)
    [~, states, addDecs] = decToMats(w);
    [N, ~] = simConstants();
    [xs, xdots, zs, zdots, ~] = extractStates(states);
    [xf, ~, ~, ~] = flightKinematics( ...
        xs(end), xdots(end), zs(end), zdots(end), addDecs(2));
    
    % swingCost = 0;
    dist = xs(end) - xs(1);
    dtravel = xf - xs(1);
    % input 'r' for running, 'h' for hopping.
    swingCost = computeSwingWorkBase(m, g, dist, addDecs(1), addDecs(2), mode, I, dtravel);
    stanceCost = trapz((addDecs(1) / (N - 1)), addDecs(3:2+N)); % fixed this
    cost = (stanceCost + swingCost);

    tstance = addDecs(1);
    tflight = addDecs(2);
end
