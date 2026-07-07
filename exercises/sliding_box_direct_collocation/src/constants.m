% constants.m
% Roger Chen
% 2026-19-06
% A place to store constants for direct collocation

function [grav, m, s_final, N, num_state_vars] = constants()
    grav = 9.81;
    m = 1;
    s_final = 10;
    N = 50;
    num_state_vars = 2;
end