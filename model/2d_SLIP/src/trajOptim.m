% trajOptim.m
% Roger Chen
% 2026-06-30
% Optimize SLIP model trajectory with fmincon
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
MODE = 'r'; % r for running, h for hopping

[m, g, k, c, la0, laRange, target_speed, target_freq, I, t_sim, l_uN] =...
    physConstants(MODE);
[N, n_states] = simConstants();

% cost function
cost = @(w) 0;                    % for debugging
cost = @(w) costFun(w, MODE);     % for COT minimzation

% bounds
% la_min = la0 - laRange;
% la_max = la0 + laRange;

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
        if MODE == 'r', load('runN35_kc275015.mat');
        else, load('hopN35v3.mat'); end
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

if MODE=='r', color='r'; else, color='b'; end
plotTraj(w_star, color, MODE);

%% printing information
disp("Current run SI units (except I and k):");
fprintf("Frequency: %.2f steps/s\n", ...
         1 / (((w_star(end-N-1) + w_star(end-N)) * t_sim)) );
unSpeed = target_speed * (l_uN / t_sim);
fprintf("Speed: %.2f m/s (%.2f mph)\n", unSpeed, unSpeed * 2.23694)
fprintf("Leg Inertia: %.3f | k:  %.3f \n", ...
         I, k);

disp("COST (" + MODE + "): ");
disp(cost(w_star));
[fAng, vAng, cAng] = getCollAngles(w_star, k, c);
disp("Collision-based analysis angles (rad): ")
fprintf("force: %f\n", fAng);
fprintf("vel: %f\n", vAng);
fprintf("coll: %f\n", cAng);

dutyFac = getDutyFactor(w_star);
fprintf("Duty Factor: %.2f\n", dutyFac);

% fprintf("True knorm (adjusted for length): %f\n", k / l_uN);
% fprintf("True cnorm (adjusted for length): %f\n", c / sqrt(l_uN));

% preserves variables
if preserveVars
    [control, states, addDecs] = decToMats(w_star);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForces(states, control, MODE);
    forces = forces / (m * g);
end

rmpath('functions/singleOpt');
rmpath('functions/sharedFuncs');
rmpath('warmStartTemplates')