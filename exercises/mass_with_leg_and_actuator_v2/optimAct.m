% optimizeAct.m
% Roger Chen
% 2026-06-17
% use fmincon to find the minimum of certain values

clear; clc; close all;

[g, Lp, m, k, c, h0, hdot0, tfinal] = constants();

%cost = @(w) w(end);                         % minimizing time but could minimize a number of things

N = 20;
%w0 = [-1 1 1 -1 1 -1 1 -1 1 -1 1 -1 1 -1 -1 -1, tfinal];                  % initial guess is all zeros
%w0 = [zeros(1, N) tfinal];
w0 = ones(1, N);
w0(1:2:end) = -1;
w0 = [w0, (tfinal - 1)];

A = [];
b = [];
Aeq = [];
beq = [];

w_min = -5;                                 % maximum actuator velocity
w_max = 5;                                  % minimum actuator velocity
lb = [(ones(1, N) * w_min), 0];
ub = [(ones(1, N) * w_max), tfinal];

options = optimoptions("fmincon", "Display", "iter", ...
    "MaxFunctionEvaluations", 15000, ...  % Default is 3000
    "MaxIterations", 1000,...
    'EnableFeasibilityMode', true, ...
    'SubproblemAlgorithm', 'cg');
w_star = fmincon(@costFun, w0, A, b, Aeq, beq, lb, ub, @nlc, options);

optimal_peaks = simulateHop(w_star);