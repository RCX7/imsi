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

    exp_pk_loc = find(act == max(act), 1);
    mdl_pk_loc = find(pred == max(pred), 1);
    % weights(pk_loc) = 5; % 10x the weight at the peak
    % weights(round(0.9 * N):end) = 5; % 5x the weight at the tail
    % weights = weights / mean(weights); % normalize

    %% compare each datapoint to compute a MSE
    mse = mean(((pred - act) .* weights).^2);
    match_peaks = 0.2 * sqrt((exp_pk_loc - mdl_pk_loc)^2 + (max(act) - max(pred))^2);
    % mse = mse + match_peaks;
end