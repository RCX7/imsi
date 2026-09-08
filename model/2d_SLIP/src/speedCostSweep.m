% speedCostSweep.m
% Roger Chen
% 2026-08-16
% Sweeps COTs across speeds. Based on analysis 1.2 of paramAnalysis (one of
% the more important analyses I the Live Script). Adapted here for speed.

clearvars; clc; %close all;

% add necessary paths
addpath('functions/multiOpt');
addpath('functions/sharedFuncs');
addpath('warmStartTemplates');

%% set up variables
addpath('functions/singleOpt');
[~, ~, k_run, c_run, ~, ~, ~, target_freq_r, I_run, ~, ~] = physConstants('r');
[~, ~, k_hop, c_hop, ~, ~, target_speed, target_freq_h, I_hop, t_sim, l_uN] = physConstants('h');
rmpath('functions/singleOpt');

target_speed = 3 / (l_uN / t_sim);  % set to a constant here.
speeds = linspace(target_speed*(1/3), target_speed*2.5, 8);

%% run cost sweep
num_speeds = length(speeds);

% 1. Preallocate matrices for parallel efficiency and sliced variable compliance
runCOTs = zeros(1, num_speeds);
run_stance_costs = zeros(1, num_speeds);
run_flight_costs = zeros(1, num_speeds);
run_stance_periods = zeros(1, num_speeds);
run_flight_periods = zeros(1, num_speeds);

hopCOTs = zeros(1, num_speeds);
hop_stance_costs = zeros(1, num_speeds);
hop_flight_costs = zeros(1, num_speeds);
hop_stance_periods = zeros(1, num_speeds);
hop_flight_periods = zeros(1, num_speeds);

% cluster = parcluster('Processes');
% cluster.NumWorkers = 24;
% saveProfile(cluster);
% parpool(num_speeds);
parfor s_idx = 1:num_speeds
    speed = speeds(s_idx);

    next_guess_run = 0;
    next_guess_hop = 0;
    
    % --- Running COT ---
    [runCOT, run_stance_cost, run_flight_cost, run_tstance, run_tflight, next_guess_run, cost_scale_factor] = ...
        robustMinCOT('r', speed, target_freq_r, k_run, c_run, I_run, next_guess_run);
    runCOTs(s_idx) = runCOT;
    run_stance_costs(s_idx) = run_stance_cost;
    run_flight_costs(s_idx) = run_flight_cost;
    run_stance_periods(s_idx) = run_tstance;
    run_flight_periods(s_idx) = run_tflight;
    
    % --- Hopping COT ---
    % Pass the preallocated slice up to the current index
    [hopCOT, hop_stance_cost, hop_flight_cost, hop_tstance, hop_tflight, next_guess_hop, cost_scale_factor] = ...
        robustMinCOT('h', speed, target_freq_h, k_hop, c_hop, I_hop, next_guess_hop);  % minCOT doubles I already
    hopCOTs(s_idx) = hopCOT;
    hop_stance_costs(s_idx) = hop_stance_cost;
    hop_flight_costs(s_idx) = hop_flight_cost;
    hop_stance_periods(s_idx) = hop_tstance;
    hop_flight_periods(s_idx) = hop_tflight;
end

%% plot
plt_speeds = speeds * (l_uN / t_sim);

figure;
subplot(1, 2, 1);
hold on;
plot(plt_speeds, runCOTs / cost_scale_factor, "r-", "LineWidth", 3);
plot(plt_speeds, run_stance_costs, "r--");
plot(plt_speeds, run_flight_costs, "r:");

plot(plt_speeds, hopCOTs / cost_scale_factor, "b-", "LineWidth", 3);
plot(plt_speeds, hop_stance_costs, "b--");
plot(plt_speeds, hop_flight_costs, "b:");

xlabel Speeds;
ylabel Cost;
legend("Run Total COT", "Run Stance COT", "Run Flight COT",...
       "Hop Total COT", "Hop Stance COT", "Hop Flight COT");
title("Cost of transport of different phases for running and hopping at different speeds");
hold off;

subplot(1, 2, 2);
hold on;
% scaled
plot(plt_speeds, (run_stance_periods + run_flight_periods) * t_sim, "r-", "LineWidth", 3);
plot(plt_speeds, run_stance_periods * t_sim, "r--");
plot(plt_speeds, run_flight_periods * t_sim, "r:");

plot(plt_speeds, (hop_stance_periods + hop_flight_periods) * t_sim, "b-", "LineWidth", 3);
plot(plt_speeds, hop_stance_periods * t_sim, "b--");
plot(plt_speeds, hop_flight_periods * t_sim, "b:");

xlabel Speeds;
ylabel("Time (seconds-scaled)");
legend("Run Total Period", "Run Stance Period", "Run Flight Period",...
       "Hop Total Period", "Hop Stance Period", "Hop Flight Period");
title("Time periods of different phases for running and hopping at different speeds");

hold off;