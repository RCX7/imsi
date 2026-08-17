% fit_kcI_wMSE.m
% Roger Chen
% 2026-07-30
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves.

clc; clearvars; %close all;

mdl_src_path = 'C:\Users\roger\imsi\model\2d_SLIP\src';
mdl_func_path = fullfile(mdl_src_path, "functions");
shared_func_path = fullfile(mdl_func_path, "sharedFuncs");
singleOpt_path = fullfile(mdl_func_path, "singleOpt");
multiOpt_path = fullfile(mdl_func_path, "multiOpt");
warmStartTemp_path = fullfile(mdl_src_path, "warmStartTemplates");

addpath(shared_func_path);
addpath(singleOpt_path);
addpath(warmStartTemp_path);
%% settings and testing conditions
gait = 'r';
unc_freq = true;
% trials = {'hop_1ms_4min'; 'run_1ms_4min'};
trials = {'hop_1ms_unc_30sec'; 'run_1ms_unc_1min'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
mse_npoints = 50;

[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, target_freq, inertia, t_sim, l_uN] =...
    physConstants(gait);

if unc_freq
    target_freq = 0;
end

% define variables
switch gait
    case 'r'
        % ks = 40:2.5:70; for constrained version
        ks = 17.5:2.5:50;
    case 'h'
        ks = 15:2.5:40;
end
Is = linspace(0.5*inertia, 2*inertia, 12);
%cs = 0.1:0.05:0.6;

% [k_mesh, c_mesh, I_mesh] = meshgrid(ks, cs, Is);
[k_mesh, I_mesh] = meshgrid(ks, Is);
k_vec = k_mesh(:);
%c_vec = c_mesh(:);
c = 0.15;  % 0.1 was pretty dang good
I_vec = I_mesh(:);


load("avgForceCurves/" + curr_trial + "_ftotal_curve", ...
    "average_stride_ftotal", "tstride_avg");  % called "average_stride_ftotal"
m = 92.65; g = 9.81;
average_stride_ftotal = average_stride_ftotal / (m * g);
num_exp_pts = numel(average_stride_ftotal);

rmpath(singleOpt_path);

%% set up output (flattened for parallelization)
n_sims = numel(k_vec);
mse_landscape = zeros(n_sims, 1);

% parallelize optimizations
addpath(multiOpt_path);
parfor i=1:n_sims
% for j=1:6
%     i = j * 28;
    k = k_vec(i);
    %c = c_vec(i);  set to constant value
    I = I_vec(i);
    
    [~, ~, ~, ~, ~, w_star] = ...
                robustMinCOT(gait, target_speed, target_freq, k, c, I, 0);
    if w_star
        [t_plot, mdl_forces, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                                w_star, average_stride_ftotal, ...
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

mse_landscape = reshape(mse_landscape, size(k_mesh));
mse_landscape(mse_landscape == 0) = max(mse_landscape, [], "all");
surf(ks, Is, mse_landscape);

%extract and plot best fit
% [best_c_idx, best_k_idx, best_I_idx] = find(mse_landscape == min(mse_landscape, [],"all"));
[best_I_idx, best_k_idx] = find(mse_landscape == min(mse_landscape, [],"all"));
best_k = ks(best_k_idx);
% best_c = cs(best_c_idx);
best_c = c;
best_I = Is(best_I_idx);
best_k_norm = best_k / l_uN;
% best_c_norm = best_c / sqrt(l_uN);

fprintf("Best fitting values of k, I: %.2f, %.2f \n", best_k, best_I);


[~, ~, ~, ~, ~, w_star] = robustMinCOT(gait, target_speed, target_freq, best_k, best_c, best_I, 0);
[t_plot, mdl_forces, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                        w_star, average_stride_ftotal, ...
                        best_k, best_c, t_sim, tstride_avg, ...
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
