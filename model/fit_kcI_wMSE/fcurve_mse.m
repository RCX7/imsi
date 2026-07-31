% fcurve_mse.m
% Roger Chen
% 2026-07-30
% Finds the mean squared error of a guess given predicted and actual
% forces.

function mse = fcurve_mse(pred, act)
    %% check that both curves have the same length going in.
    assert(numel(pred) == numel(act));

    %% compare each datapoint to compute a MSE
    mse = mean((pred - act).^2);
end