% detailedCost.m
% Roger Chen
% 2026-07-07
% Gives a detailed report on the statistics of the cost function, including
% stance and flight cost, and stance and flight times

function [cost, stanceCost, flightCost, tstance, tflight]...
    = detailedCost(w, mode, I)

    [u, states, addDecs] = decToMats(w);
    [N, n_states] = simConstants();
    [xs, xdots, zs, zdots, las] = extractStates(states);
    [xf, xdotf, zf, zdotf] = flightKinematics( ...
        xs(end), xdots(end), zs(end), zdots(end), addDecs(2));
    
    % swingCost = 0;
    dist = xs(end) - xs(1);
    dtravel = xf - xs(1);
    % input 'r' for running, 'h' for hopping.
    flightCost = computeSwingWork(dist, addDecs(1), addDecs(2), mode, I, dtravel);   
    stanceCost = trapz(addDecs(1), addDecs(3:2+N));
    cost = stanceCost + flightCost;
    tstance = addDecs(1);
    tflight = addDecs(2);
end