% extractStates.m
% Roger Chen
% 2026-06-30
% A helper to extract specific states (for more readable code) from state
% matrix. Should be updated with additional states

function [xs, xdots, zs, zdots, las] = extractStates(states)
    [N, n_states] = simConstants();   % sanity check/safeguard
    assert(all(size(states) == [n_states, N]));

    xs = states(1,:);
    xdots = states(2,:);
    zs = states(3,:);
    zdots = states(4,:);
    las = states(5,:);
end