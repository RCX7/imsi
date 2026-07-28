% detailedCost.m
% Roger Chen
% 2026-07-07
% Gives a detailed report on the statistics of the cost function, including
% stance and flight cost, and stance and flight times

function [cost, stanceCost, swingCost, tstance, tflight]...
    = detailedCost(w, mode, I)

    [m, g, ~] = physConstants();
    [cost, stanceCost, swingCost, tstance, tflight] = costFunBase(w, m, g, I, mode);

end