% computeSwingWork.m
% Roger Chen
% Computes the work required to move the swing leg forward, effectively
% adding leg inertia to the optimal control problem.

function work = computeSwingWork(dist, ts, tf, mode, dtravel)

    [m, g, ~, ~, ~, ~, ~, ~, I, ~, ~] = physConstants(mode);
    work = computeSwingWorkBase(m, g, dist, ts, tf, mode, I, dtravel);
end