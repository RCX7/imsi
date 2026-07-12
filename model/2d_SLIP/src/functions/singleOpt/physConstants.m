% physCconstants.m
% Roger Chen
% 2026-06-30
% Modularizing commonly used physicaal constants. Used in determining the
% kinematics of the simulation

function [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode)
    assert(mode == 'r' | mode == 'h')
    
    m = 1;
    %g = 9.81;
    g = 1;
    k = 15;
    c = 0.4;
    la0 = 1;

    % k = 955.1842;
    % c = 3.883;
    % la0 = 0.228;
    
    laRange = 0.2; % controls how much increase or decrease the leg can have
    xdot_target = 2;
    % I = 1.8e-1;
    I = 0.05;

    if mode == 'h'
        k = k*2;
        c = c*2;
        I = I*2;
    end
end