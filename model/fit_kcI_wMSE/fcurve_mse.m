% fcurve_mse.m
% Roger Chen
% 2026-07-30
% Finds the mean squared error of a guess given predicted and actual
% forces.

function mse = fcurve_mse(pred, act)
    %% check that both curves have the same length going in.
    assert(numel(pred) == numel(act));
    
    % TODO: UPWEIGHT LAST PARTS OF THE CURVE (THE TAIL) TO ACCURATELY FIT
    % DAMPING!
    N = numel(pred);
    weights = ones(1, N);
    weights(round(0.9 * N):end) = 3; % triple the weight at the tail

    %% compare each datapoint to compute a MSE
    mse = mean(((pred - act) .* weights).^2);
end