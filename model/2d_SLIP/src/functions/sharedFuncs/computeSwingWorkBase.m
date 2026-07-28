% computeSwingWorkBase.m
% Roger Chen
% 2026-07-28
% Computes the cost of the swing phase of gait given some parameters.

function work = computeSwingWorkBase(m, g, dist, ts, tf, mode, I, dtravel)
    switch mode
        case 'r' % running
            work = I * (2 * ((dist) / (2*tf + ts)))^2;
        case 'h' % hopping
            work = I * (2 * ((dist) / (tf)))^2;
    end
    work = work / (m * g * dtravel);
end