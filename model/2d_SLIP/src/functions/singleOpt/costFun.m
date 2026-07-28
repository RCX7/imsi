% costFun.m
% Roger Chen
% 2026-06-30
% A cost function to minimize energy expenditure for trajectory
% optimization

function [cost] = costFun(w, mode)
    [m, g, ~, ~, ~, ~, ~, ~, I, ~, ~] = physConstants(mode);
    [cost, ~, ~, ~, ~] = costFunBase(w, m, g, I, mode);
end