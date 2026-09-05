% fit_kcI_wMSE.m
% Roger Chen
% 2026-07-30
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves.

clc; clearvars; %close all;

% mdl_src_path = 'C:\Users\azizi\Downloads\rogerc8_imsi_work\imsi\model\2d_SLIP\src';
% mdl_src_path = 'C:\Users\roger\imsi\model\2d_SLIP\src';
mdl_src_path = 'C:\Users\Public\Documents\Roger\imsi\model\2d_SLIP\src';
mdl_func_path = fullfile(mdl_src_path, "functions");
shared_func_path = fullfile(mdl_func_path, "sharedFuncs");
singleOpt_path = fullfile(mdl_func_path, "singleOpt");
multiOpt_path = fullfile(mdl_func_path, "multiOpt");
warmStartTemp_path = fullfile(mdl_src_path, "warmStartTemplates");

addpath(shared_func_path);
addpath(singleOpt_path);
addpath(warmStartTemp_path);
%% settings and testing conditions
subject = "S05";
m = 91.35; g = 9.81;
gait = 'h';
unc_freq = true;
% trials = {'hop_1ms_4min'; 'run_1ms_4min'};
% trials = {'hop_1ms_unc_1min'; 'run_1ms_unc_1min'};
trials = {'hop_2ms'; 'run_2ms'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
mse_npoints = 150;

[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, target_freq, ~, t_sim, l_uN] =...
    physConstants(gait);

if unc_freq
    target_freq = 0;
end

% define variables
switch gait
    case 'r'
        % ks = 40:2.5:70; for constrained version
        ks = 10:2.5:30;
        Is = 0.005:0.005:0.06;
    case 'h'
        ks = 20:2.5:50;
        Is = 0.01:0.01:0.09;
end

cs = 0.025:0.05:0.5;

[k_mesh, c_mesh, I_mesh] = meshgrid(ks, cs, Is);
k_vec = k_mesh(:);
c_vec = c_mesh(:);
I_vec = I_mesh(:);

trial_timing = "full_stride";
load("avgForceCurves/" + subject + "/" + trial_timing + "/" + subject + "_" + curr_trial + "_ftotal_curve", ...
    "average_stride_ftotal", "tstance_avg", "tflight_avg", "tstride_avg");  % called "average_stride_ftotal"
average_stride_ftotal = average_stride_ftotal / (m * g);  % scale by BW
num_exp_pts = numel(average_stride_ftotal);

rmpath(singleOpt_path);

%% set up output (flattened for parallelization)
n_sims = numel(k_vec);
mse_landscape = zeros(n_sims, 1);

% parallelize optimizations
addpath(multiOpt_path);
% cluster = parcluster('local'); 
% cluster.NumWorkers = 24; 
% saveProfile(cluster); 
% parpool(24); 
parfor i=1:n_sims
    k = k_vec(i);
    c = c_vec(i);
    I = I_vec(i);
    % k = 35;
    % c = 0.2;
    % I = 0.1;
    
    [~, ~, ~, ~, ~, w_star] = ...
                robustMinCOT(gait, target_speed, target_freq, k, c, I, 0);

    if w_star
        [control, states, addDecs] = decToMats(w_star);
        mdl_forces = computeForces(states, control, k, c);

        mdl_tstance = addDecs(1) * t_sim;
        mdl_tflight = addDecs(2) * t_sim;
        if gait == 'r'  % to match with experimental force curve, which has another stride in between
            mdl_tflight = mdl_tflight + ((addDecs(1) + addDecs(2)) * t_sim);
        end
        [~, mdl_forces_itrp, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                                mdl_forces, average_stride_ftotal, ...
                                mdl_tstance, mdl_tflight, tstance_avg, tflight_avg, ...
                                num_mdl_pts, num_exp_pts, mse_npoints ...
                            );
    
        %% measure MSE and store
        mse = fcurve_mse(mdl_forces_itrp, exp_forces);
        mse_landscape(i) = mse;
    end
end

%% reshape landscape and plot slices
mse_landscape = reshape(mse_landscape, size(k_mesh));
mse_landscape(mse_landscape == 0) = max(mse_landscape, [], "all");
mse_landscape(mse_landscape > 5) = 5;

num_subplots = numel(Is);
% num_cols = 4;
% num_rows = ceil(num_subplots / num_cols);

for i=1:num_subplots
    curr_slice = mse_landscape(:,:,i);
    curr_I = Is(i);
    
    figure;
    % subplot(num_rows, num_cols, i);
    surf(ks, cs, curr_slice);
    title(sprintf('MSE Landscape Slice for I = %.5f', curr_I));
    xlabel('Spring Constant (k)');
    ylabel('Damping Constant (c)');
    zlabel('Mean Squared Error (MSE)');
end



%% extract and plot best fit
linear_idx = find(mse_landscape == min(mse_landscape, [], "all"));
[best_c_idx, best_k_idx, best_I_idx] = ind2sub(size(mse_landscape), linear_idx);
best_k = ks(best_k_idx);
best_c = cs(best_c_idx);
best_I = Is(best_I_idx);

fprintf("Best fitting values of k, c, I: %.2f, %.2f \n", best_k, best_c, best_I);


[~, ~, ~, ~, ~, w_star] = ...
                robustMinCOT(gait, target_speed, target_freq, best_k, best_c, best_I, 0);

[control, states, addDecs] = decToMats(w_star);
mdl_forces = computeForces(states, control, best_k, best_c);

mdl_tstance = addDecs(1) * t_sim;
mdl_tflight = addDecs(2) * t_sim;
if gait == 'r'
    mdl_tflight = mdl_tflight + ((addDecs(1) + addDecs(2)) * t_sim);
end
[t_plot, mdl_forces_itrp, exp_forces] = norm_and_plot_mdl_vs_exp( ...
                                mdl_forces, average_stride_ftotal, ...
                                mdl_tstance, mdl_tflight, tstance_avg, tflight_avg, ...
                                num_mdl_pts, num_exp_pts, mse_npoints ...
                            );

figure;
title("Best fitting plot");
hold on;
plot(t_plot, mdl_forces_itrp);
plot(t_plot, exp_forces);
legend(["Model force", "Exp Force"]);
xlabel("Time (secs)");
ylabel("Force (BWs)");
hold off;

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);
