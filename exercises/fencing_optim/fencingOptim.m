% fencingOptim.m
% Roger Chen
% 2026-06-17
% quick practice with fmincon

clc
clear
close all

cost = @(v) -(v(1) * v(2));    % negative area
A = [-1 0; 0 -1];
b = [0; 0];                   % say that L is limited

Aeq = [2 2];
beq = 40;

optSoln = fmincon(cost, [10 30], A, b, Aeq, beq)

%% now with nonlcon
function [g, geq] = nlc(w)           % returns [g, geq]
    g = 2 * w(1) + 2 * w(2) - 40;
    geq = 0;
end

% sensitive to bad guesses
optSoln2 = fmincon(cost, [10 30], [], [], [], [], [0; 0], [], @nlc)




