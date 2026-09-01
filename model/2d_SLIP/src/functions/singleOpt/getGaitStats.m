function [COT, stride_freq, stride_len, stride_duration, dutyfac, peak_force, impulse, fAng, vAng, cAng] ...
    = getGaitStats(w_star, gait, output_data)
%getGaitStats - single optimization version of getting gait characteristics
% (e.g. stride frequency, length, peak force...)
%  calls physConstants to get variables
    [m, g, k, c, ~, ~, ~, ~, I, t_sim, l_uN]...
                = physConstants(gait);

    [COT, stride_freq, stride_len, stride_duration, dutyfac, peak_force, impulse, fAng, vAng, cAng] =...
        getGaitStatsBase(w_star, gait, m, g, k, c, I, t_sim, l_uN, output_data);
end

