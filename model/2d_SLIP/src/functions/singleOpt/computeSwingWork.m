% computeSwingWork.m
% Roger Chen
% Computes the work required to move the swing leg forward, effectively
% adding leg inertia to the optimal control problem.

function work = computeSwingWork(dist, ts, tf, mode, dtravel)

    [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode);
    switch mode
        case 'r' % running
            work = I * ((dist) / (2*tf + ts))^2;
        case 'h' % hopping
            work = I * ((dist) / (tf))^2;
    end
    work = work / (m * g * dtravel);
end