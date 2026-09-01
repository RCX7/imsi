% mdl_exp_compare.m
% Roger Chen
% 09-10-2026
% Similar to trajOpt.m, but extracts from exp_data and compares model
% results with the experiments

addpath('functions/singleOpt');
addpath('functions/sharedFuncs');
addpath('warmStartTemplates');

% use a previous solution as the starting guess
warmStart = true;

clc; %close all;
if warmStart
    clearvars -except w_star warmStart subjectData;
else
    clearvars -except warmStart subjectData;
end

preserveVars = true;
A = []; b = []; Aeq = []; beq = [];

% get constants
MODE = 'h'; % r for running, h for hopping

[m, g, k, c, la0, laRange, target_speed, target_freq, I, t_sim, l_uN] =...
    physConstants(MODE);
[N, n_states] = simConstants();

% cost function
cost = @(w) costFun(w, MODE);     % for COT minimization

lb_u = -inf(1, N);
lb_states = -inf(n_states, N);      % x, xdot, zdot can be arbitrarily low
lb_states(3,:) = 0;                 % z - cannot fall through floor
lb_addDecs = [0, 0, zeros(1, N)];       % time and magnitude of power
lb = matToDec(lb_u, lb_states, lb_addDecs);

ub_u = inf(1, N);
ub_states = inf(n_states, N);
ub_addDecs = [inf, inf, inf(1, N)];
ub = matToDec(ub_u, ub_states, ub_addDecs);

% decision vector (inital guess)
if warmStart
    if ~exist('w_star', 'var')
        disp("using warm start template");
        if MODE == 'r', load('warmStartTemplates/runN35_kc275015_fast.mat');
        else, load('warmStartTemplates/hopN35_kc2001.mat'); end
    end
    
    if numel(w_star) ~= ((n_states + 2) * N + 2)  % scale to different number of points
        [u, states, addDec] = decToMats(w_star, 35);
        u = interp1(1:35, u, linspace(1, 35, N));
        states = interp1(1:35, states', linspace(1, 35, N))';
        new_addDecs = [addDec(1) addDec(2) zeros(1, N)];
        new_addDec(3:N+2) = interp1(1:35, addDec(3:end), linspace(1, 35, N));
        w_star = matToDec(u, states, new_addDec);
    end
    w0 = w_star;
else  % manual initialization
    u = zeros(N, 1);
    % states = zeros(n_states, N) + 0.1;
    states(1,:) = linspace(-0.015, 0.025, N);
    states(1,:) = 0;
    states(2,:) = 1;
    % states(3,:) = [linspace(1, 0.5, N/2), linspace(0.5, 1, N/2 + 1)];
    states(4, :) = -1;
    states(5,:) = 0.1;    
    addDecs = [0.05; 0.0; zeros(N, 1) + 0.1];  % ts, tf, power (abs, N terms)
    w0 = matToDec(u, states, addDecs);
end

options = optimoptions("fmincon", "Display", "iter",...
    "MaxFunctionEvaluations",80000, "MaxIterations",1000);
w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub,...
    @(w) trajConstraints(w, MODE), options);

if MODE=='r', color='r'; else, color='g'; end
figure;
plotTraj(w_star, color, MODE);

%% comparing gait parameters
[mdl_COT, mdl_stride_freq, mdl_stride_len, mdl_stride_duration, mdl_dutyfac, mdl_peak_force, mdl_impulse, mdl_fAng, mdl_vAng, mdl_cAng] ...
    = getGaitStats(w_star, MODE, true);

% adjust to match experiments for running (considers two steps a stride)
if MODE == 'r'
    mdl_stride_freq = mdl_stride_freq / 2;
    mdl_stride_len = mdl_stride_len * 2;
    mdl_peak_force = mdl_peak_force * 2;
    mdl_impulse = mdl_impulse * 2;
end

% load experimental data into variable 'subjectData'
subject = "S05";
BW = 59.12;
trial_speed = target_speed * (l_uN / t_sim);
load("exp_data/" +subject + "/" + subject + "_results_summary_" + trial_speed + "ms.mat");
if MODE == 'r'
    trial_data = subjectData.run_1ms_unc_1min;
elseif MODE == 'h'
    trial_data = subjectData.hop_1ms_unc_30sec;
end

exp_stride_freq = trial_data.General.StrideFrequency;
exp_dutyfac = trial_data.General.DutyFactor;
exp_stride_len = trial_data.General.StrideLength;
exp_peak_force = trial_data.Force.AvgPeakForceFz / (BW * 9.81);
exp_impulse = trial_data.Force.AvgImpulseFz / (BW * 9.81);
exp_fAng = trial_data.CollisionAngles.ForceAngle;
exp_vAng = trial_data.CollisionAngles.VelocityAngle;
exp_cAng = trial_data.CollisionAngles.CollisionAngle;
exp_COT = trial_data.CollisionAngles.CoTmech;

% coalesce data

param_labels = [
    "COT";
    "Stride Frequency";
    "Stride Length";
    "Duty Factor";
    "Peak Force";
    "Impulse";
    "Force Angle";
    "Velocity Angle";
    "Collision Angle";
];

mdl_params = [
    mdl_COT;
    mdl_stride_freq;
    % mdl_stride_duration;
    mdl_stride_len;
    mdl_dutyfac;
    mdl_peak_force;
    mdl_impulse;
    mdl_fAng;
    mdl_vAng;
    mdl_cAng;
];

exp_params = [
    exp_COT;
    exp_stride_freq;
    % exp_stride_duration;
    exp_stride_len;
    exp_dutyfac;
    exp_peak_force;
    exp_impulse;
    exp_fAng;
    exp_vAng;
    exp_cAng;
];

figure;
bar([mdl_params exp_params]);
xticklabels(param_labels);
legend('model', 'experiment');


% preserves variables if needed
if preserveVars
    [control, states, addDecs] = decToMats(w_star);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForces(states, control, MODE);
    forces = forces / (m * g);
end

rmpath('functions/singleOpt');
rmpath('functions/sharedFuncs');
rmpath('warmStartTemplates')

