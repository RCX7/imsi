function [COT, stride_freq, stride_len, stride_duration, dutyfac, peak_force, impulse, fAng, vAng, cAng] ...
    = getGaitStats(w_star, gait, k, c, I, t_sim, l_uN, output_data)
%getGaitStats - multiple optimization version of getting gait characteristics
% (e.g. stride length, frequency, peak force, etc.)
%  calls physConstants to get variables but gets rest from inputs

    [m, g, ~] = physConstants();

    [COT, stride_freq, stride_len, stride_duration, dutyfac, peak_force, impulse, fAng, vAng, cAng] =...
        getGaitStatsBase(w_star, gait, m, g, k, c, I, t_sim, l_uN, output_data);

end

