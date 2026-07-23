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
    % k = 42.389;  % running fit
    k = 57.5;
    c = 0.3;
    la0 = 1;
    
    laRange = 10; % controls how much increase or decrease the leg can have
    % xdot_target = 1;
    xdot_target = 0.3;
    % xdot_target = 0.6;
    I = 0.0463;

    if mode == 'h'
        k = 21.9081;  % hopping fit
        c = 0.3;
        I = I*2;
    end
end