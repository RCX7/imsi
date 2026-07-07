% computeSwingWork.m
% Roger Chen
% Computes the work required to move the swing leg forward, effectively
% adding leg inertia to the optimal control problem.

function work = computeSwingWork(dist, ts, tf, mode, I)
    switch mode
        case 'r' % running
            work = I * ((2*dist) / (2*tf + ts))^2;
        case 'h' % hopping
            work = I * ((dist) / (tf))^2;
    end
end