% fit_kcI_wMSE.m
% Roger Chen
% 2026-07-30
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves.

clc; clearvars; %close all;

mdl_src_path = 'C:\Users\azizi\Downloads\rogerc8_imsi_work\imsi\model\2d_SLIP\src';
mdl_func_path = fullfile(mdl_src_path, "functions");
shared_func_path = fullfile(mdl_func_path, "sharedFuncs");
singleOpt_path = fullfile(mdl_func_path, "singleOpt");
multiOpt_path = fullfile(mdl_func_path, "multiOpt");
warmStartTemp_path = fullfile(mdl_src_path, "warmStartTemplates");

addpath(shared_func_path);
addpath(singleOpt_path);
addpath(warmStartTemp_path);
%% settings and testing conditions
subject = "S06";
m = 59.12; g = 9.81;  % MAKE SURE TO: adjust mass
gait = 'r';
unc_freq = true;
% trials = {'hop_1ms_4min'; 'run_1ms_4min'};
trials = {'hop_1ms_unc_1min'; 'run_1ms_unc_1min'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
mse_npoints = 50;

[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, target_freq, inertia, t_sim, l_uN] =...
    physConstants(gait);

if unc_freq
    target_freq = 0;
end

%% define variables to sweep across
switch gait
    case 'r'
        % ks = 40:2.5:70; for constrained version
        ks = 15:2.5:35;
    case 'h'
        ks = 10:2.5:35;
end
Is = linspace(0.5*inertia, 2*inertia, 12);
cs = 0.025:0.025:0.6;

first_var = ks;
second_var = cs;

[firstvar_mesh, secondvar_mesh] = meshgrid(first_var, second_var);
firstvar_vec = firstvar_mesh(:);
secondvar_vec = secondvar_mesh(:);

load("avgForceCurves/" + subject + "/" + subject + "_" + curr_trial + "_ftotal_curve", ...
    "average_stride_ftotal", "tstride_avg");  % called "average_stride_ftotal"
average_stride_ftotal = average_stride_ftotal / (m * g);
num_exp_pts = numel(average_stride_ftotal);

rmpath(singleOpt_path);

%% set up output (flattened for parallelization)
n_sims = numel(firstvar_vec);
mse_landscape = zeros(n_sims, 1);

% parallelize optimizations
addpath(multiOpt_path);
% cluster = parcluster('local'); 
% cluster.NumWorkers = 24; % on desktop workstation
% saveProfile(cluster); 
% parpool(24);
%% define unswept variables -- adjust this --
if gait == 'r', I = 0.04; else I = 0.08; end

parfor i=1:n_sims
% for j=1:6
%     i = j * 28;
    k = firstvar_vec(i);
    c = secondvar_vec(i);
    
    [~, ~, ~, ~, ~, w_star] = ...
                robustMinCOT(gait, target_speed, target_freq, k, c, I, 0);

    [control, states, ~] = decToMats(run_w_star);
    mdl_forces = computeForces(states, control, k, c);

    if w_star
        [~, mdl_forces_itrp, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                                mdl_forces, average_stride_ftotal, ...
                                k, c, t_sim, tstride_avg, ...
                                num_mdl_pts, num_exp_pts, mse_npoints ...
                            );
    
        %% measure MSE and store
        mse = fcurve_mse(mdl_forces, exp_forces);
        mse_landscape(i) = mse;
    end
    % subplot(2, 3, j);
    % title("Index: " + i + " k: " + k + " c: " + c  + " I: " + I + " mse: " + mse);
    % hold on;
    % plot(t_plot, mdl_forces);
    % plot(t_plot, exp_forces);
    % xlabel("Time (secs)");
    % ylabel("Force (BWs)");
    % hold off;
end

mse_landscape = reshape(mse_landscape, size(firstvar_mesh));
mse_landscape(mse_landscape == 0) = max(mse_landscape, [], "all");
mse_landscape(mse_landscape > 2) = 2;
figure;
surf(ks, cs, mse_landscape);

%extract and plot best fit
[best_secondvar_idx, best_firstvar_idx] = find(mse_landscape == min(mse_landscape, [],"all"));
best_firstvar = first_var(best_firstvar_idx);
best_secondvar = second_var(best_secondvar_idx);

% --- adjust this --- %
best_k = best_firstvar;
best_c = best_secondvar;
best_I = I ;

fprintf("Best fitting values of k, c: %.2f, %.2f \n", best_firstvar, best_c);


[~, ~, ~, ~, ~, w_star] = robustMinCOT(gait, target_speed, target_freq, best_k, best_c, best_I, 0);
[t_plot, mdl_forces, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                        w_star, average_stride_ftotal, ...
                        best_firstvar, best_secondvar, t_sim, tstride_avg, ...
                        num_mdl_pts, num_exp_pts, mse_npoints ...
                    );
% some code duplication here ... to be fixed
figure;
title("Best fitting plot");
hold on;
plot(t_plot, mdl_forces);
plot(t_plot, exp_forces);
xlabel("Time (secs)");
ylabel("Force (BWs)");
hold off;

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);
