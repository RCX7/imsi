% computeSwingWork.m
% Roger Chen
% Computes the work required to move the swing leg forward, effectively
% adding leg inertia to the optimal control problem.

function work = computeSwingWork(dist, ts, tf, mode, I, dtravel)
    [m, g, la0] = physConstants();
    switch mode
        case 'r' % running
            work = I * ((dist) / (2*tf + ts))^2;
        case 'h' % hopping
            work = I * ((dist) / (tf))^2;
    end
    work = work / (m * g * dtravel * (ts + tf));
end