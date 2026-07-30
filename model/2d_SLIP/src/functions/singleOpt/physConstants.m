% physCconstants.m
% Roger Chen
% 2026-06-30
% Modularizing commonly used physicaal constants. Used in determining the
% kinematics of the simulation

function [m, g, k, c, la0, laRange, target_speed, target_freq, I, t_sim, l_uN]...
            = physConstants(mode)
    assert(mode == 'r' | mode == 'h')
    
    %% UNNORMALIZED DATA
    g_uN = 9.81;
    % l_uN = 1.15;  % distance from COM to COP in meters
    l_uN = 0.9;     % hip height in m
    target_speed_uN = 2;  % in meters per second
    target_freq_uN = 0;   % set to 0 to unconstrain
    if mode == 'r', target_freq_uN = target_freq_uN * 2; end

    %% NORMALIZED DATA
    m = 1;
    g = 1;
    k=25;       % experiment with k = 25 for both
    % k = 28.028;
    % k = 30.238;
    % k = 42.389;  % running fit
    % k = 50;
    % k = 62.055;        % vertical stiffness fit
    c = 0.3;
    la0 = 1;
    t_sim = sqrt(l_uN / g_uN);  % unit of simulation time, in seconds
    target_freq = target_freq_uN * t_sim;
    
    laRange = 10; % unused - probably unimportant
    target_speed = target_speed_uN * (t_sim / l_uN);
    I = 0.0463; % maybe normalize somehow
    % I = 0.08;

    if mode == 'h'
        k = 18;
        % k = 16.677;  % method e fit
        % k = 22.141;
        % k = 31.0771;   % vertical stiffness fit
        c = 0.3;
        I = I*2;
    end
end