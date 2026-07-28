% costFun.m
% Roger Chen
% 2026-06-30
% A cost function to minimize energy expenditure for trajectory
% optimization

function [cost] = costFun(w, mode, I)
    [m, g, ~] = physConstants();
    [cost, ~, ~, ~, ~] = costFunBase(w, m, g, I, mode);
end