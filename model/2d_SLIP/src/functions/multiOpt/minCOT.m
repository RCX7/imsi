% minCOT.m
% Roger Chen
% 2026-07-07
% A callable module based off of a single trajectory optimization.
% Minimizes the COT for certain input parameters.
% for wide parameter sweeps.
% INPUTS:
%   - mode ('r' for running, 'h' for hopping)
%   - parameters of interest:
%       - speed
%       - leg inertia
%       - spring constant
%       - damping constant
% OUTPUTS:
%   - cost of transport (with leg inertia)

function [COT, stanceCost, flightCost, tstance, tflight, w_star] =...
    minCOT(mode, target_speed, target_freq, k, c, I, init_guess)

    arguments
        mode char
        target_speed
        target_freq
        k
        c
        I
        init_guess = 0;
    end
    fprintf("Mode: %s | Speed: %.2f | k, c, I: %d %.2f %.5f\n", mode, target_speed, k, c, I);

    assert (mode == 'h' | mode == 'r');
    clearvars -except w_star mode target_speed target_freq k c I init_guess;

    A = []; b = []; Aeq = []; beq = [];
    
    % get constants
    [N, n_states] = simConstants();
    % if mode == 'h'
    %     I = I * 2.5;
    %     % target_freq = target_freq * 2;
    % end
    
    % cost function and constraints
    cost = @(w) costFun(w, mode, I);     % for energy minimzation
    constraints = @(w) trajConstraints(w, target_speed, target_freq, k, c);
    
    % define bounds
    lb_u = -inf(1, N);
    lb_states = -inf(n_states, N);      % x, xdot, zdot can be arbitrarily low
    lb_states(3,:) = 0;                 % z - cannot fall through floor
    lb_addDecs = [0, 0, zeros(1, N)];       % time and magnitude of power
    lb = matToDec(lb_u, lb_states, lb_addDecs);
    
    ub_u = inf(1, N);
    ub_states = inf(n_states, N);
    ub_addDecs = [inf, inf, inf(1, N)];
    ub = matToDec(ub_u, ub_states, ub_addDecs);

    
    % decision vector (inital guess)
    if init_guess == 0
        disp("using warm start template");
        if mode == 'r', load('warmStartTemplates/runN35_kc275015_fast.mat');
        else, load('warmStartTemplates/hopN35_kc2001.mat'); end
        w0 = w_star;
    else
        w0 = init_guess;
    end
    
    options = optimoptions("fmincon", "Display", "notify-detailed", ...
        "MaxFunctionEvaluations",80000, "MaxIterations",1000);
    w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub,...
        constraints, options);
    
    [COT, stanceCost, flightCost, tstance, tflight]...
    = detailedCost(w_star, mode, I);
end