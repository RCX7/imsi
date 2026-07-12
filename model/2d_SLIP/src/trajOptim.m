% trajOptim.m
% Roger Chen
% 2026-06-30
% Optimize SLIP model trajectory with fmincon
addpath('functions/singleOpt');
addpath('functions/sharedFuncs');
addpath('warmStartTemplates');

% use a previous solution as the starting guess
warmStart = true;

clc; %close all;
if warmStart
    clearvars -except w_star warmStart;
else
    clearvars -except warmStart;
end

A = []; b = []; Aeq = []; beq = [];

% get constants
MODE = 'r'; % r for running, h for hopping

[m, g, k, c, la0, laRange, xdot_target, I] = physConstants(MODE);
[N, n_states] = simConstants();

% cost function
cost = @(w) 0;              % for debugging
cost = @(w) costFun(w, MODE);     % for energy minimzation

% bounds
la_min = la0 - laRange;
la_max = la0 + laRange;

lb_u = -inf(1, N);
lb_states = -inf(n_states, N);      % x, xdot, zdot can be arbitrarily low
lb_states(3,:) = 0;                 % z - cannot fall through floor
% lb_states(5,:) = la_min;            % la - some range of motion
lb_addDecs = [0, 0, zeros(1, N)];       % time and magnitude of power
lb = matToDec(lb_u, lb_states, lb_addDecs);

ub_u = inf(1, N);
ub_states = inf(n_states, N);
% ub_states(5,:) = la_max;
ub_addDecs = [inf, inf, inf(1, N)];
ub = matToDec(ub_u, ub_states, ub_addDecs);

% decision vector (inital guess)
if warmStart
    if ~exist('w_star', 'var')
        disp("using warm start template");
        if MODE == 'r', load('runWarmStart.mat');
        else, load('hopWarmStart.mat'); end
    end
    w0 = w_star;
else
    u = zeros(N, 1) + 1;
    states = zeros(n_states, N) + 0.1;
    states(1,:) = linspace(-0.015, 0.025, N);
    states(3,:) = 0.18;
    states(4, :) = -1;
    states(5,:) = 0.2;    
    addDecs = [0.04; 0.01; zeros(N, 1) + 0.3];  % ts, tf, power (abs, N terms)
    w0 = matToDec(u, states, addDecs);
end

options = optimoptions("fmincon", "Display", "iter",...
    "MaxFunctionEvaluations",200000, "MaxIterations",1000);
w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub,...
    @(w) trajConstraints(w, MODE), options);

%plotTraj(w_star, "b", MODE);
disp("COST (" + MODE + "): ");
disp(cost(w_star));

rmpath('functions/singleOpt');
rmpath('functions/sharedFuncs');
rmpath('warmStartTemplates')