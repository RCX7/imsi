% fit_kcI_parRunHop_3var.m
% Roger Chen
% 2026-08-30
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves. OPERATES UNDER THE ASSUMPTION / CONSTRAINT THAT
% HOPPING HAS 2x SPRING, DAMPING, AND INERTIA

clc; clearvars; %close all;

% mdl_src_path = 'C:\Users\azizi\Downloads\rogerc8_imsi_work\imsi\model\2d_SLIP\src';
mdl_src_path = 'C:\Users\roger\imsi\model\2d_SLIP\src';
% mdl_src_path = 'C:\Users\Public\Documents\Roger\imsi\model\2d_SLIP\src';
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

% trials = {'run_1ms_4min'; 'hop_1ms_4min'};
% trials = {'run_1ms_unc_1min'; 'hop_1ms_unc_1min'};
trials = {'run_2ms'; 'hop_2ms'};

mse_npoints = 50;

[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, target_freq, ~, t_sim, l_uN] =...
    physConstants('r');  % always take running parameters.


%% define variables to sweep across
ks = 15:2.5:35;
Is = 0.01:0.01:0.07;
cs = 0.025:0.025:0.6;

[k_mesh, c_mesh, I_mesh] = meshgrid(ks, cs, Is);
k_vec = k_mesh(:);
c_vec = c_mesh(:);
I_vec = I_mesh(:);

load("avgForceCurves/" + subject + "/" + subject + "_" + trials{1} + "_ftotal_curve", ...
    "average_stride_ftotal", "tstride_avg");  % called "average_stride_ftotal"
run_tstride_avg = tstride_avg;
run_average_stride_ftotal = average_stride_ftotal / (m * g);
% num_exp_pts_run = numel(average_stride_ftotal_run);  pretty sure this is 

load("avgForceCurves/" + subject + "/" + subject + "_" + trials{2} + "_ftotal_curve", ...
    "average_stride_ftotal", "tstride_avg");  % called "average_stride_ftotal"
hop_average_stride_ftotal = average_stride_ftotal / (m * g);
hop_tstride_avg = tstride_avg;
num_exp_pts = numel(average_stride_ftotal);

rmpath(singleOpt_path);

%% set up output (flattened for parallelization)
n_sims = numel(k_vec);
mse_landscape = zeros(n_sims, 1);

% parallelize optimizations
addpath(multiOpt_path);
% cluster = parcluster('local'); 
% cluster.NumWorkers = 24; % on desktop workstation
% saveProfile(cluster); 
% parpool(24);

parfor i=1:n_sims
    k = k_vec(i);
    c = c_vec(i);
    I = I_vec(i);
    
    [~, ~, ~, ~, ~, run_w_star] = ...
                robustMinCOT('r', target_speed, target_freq, k, c, I, 0);
    [~, ~, ~, ~, ~, hop_w_star] = ...
                robustMinCOT('h', target_speed, target_freq, 2*k, 2*c, 2*I, 0);

    [run_control, run_states, run_addDecs] = decToMats(run_w_star);
    run_mdl_forces_itrp = computeForces(run_states, run_control, k, c);
    [hop_control, hop_states, ~] = decToMats(hop_w_star);
    hop_mdl_forces = computeForces(hop_states, hop_control, 2*k, 2*c);
    % assumes hopping has double the spring constant

    if all(~isinf(run_w_star)) && all(~isinf(hop_w_star))
        [~, run_mdl_forces_itrp, run_exp_forces] = norm_and_plot_mdl_vs_exp( ...
                                run_mdl_forces, run_average_stride_ftotal, ...
                                run_addDecs(1), 0, t_sim, run_tstride_avg, ...
                                num_mdl_pts, num_exp_pts, mse_npoints ...
                            );
        [~, hop_mdl_forces_itrp, hop_exp_forces] = norm_and_plot_mdl_vs_exp( ...
                            hop_mdl_forces, hop_average_stride_ftotal, ...
                            hop_addDecs(1), 0, t_sim, hop_tstride_avg, ...
                            num_mdl_pts, num_exp_pts, mse_npoints ...
                        );
    
        %% measure MSE and store
        mse = fcurve_mse(run_mdl_forces_itrp, run_exp_forces) + ...
            fcurve_mse(hop_mdl_forces_itrp, hop_exp_forces);
        mse_landscape(i) = mse;
    end
end

%% reshape landscape and plot slices
mse_landscape = reshape(mse_landscape, size(k_mesh));
mse_landscape(mse_landscape == 0) = max(mse_landscape, [], "all");
mse_landscape(mse_landscape > 1) = 1;

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

fprintf("Best fitting values of k, c, I: %.2f, %.2f, %.2f \n", best_k, best_c, best_I);


[~, ~, ~, ~, ~, run_w_star] = robustMinCOT('r', target_speed, target_freq, best_k, best_c, best_I, 0);
[run_t_plot, run_mdl_forces, run_exp_forces] = norm_and_plot_mdl_vs_exp( ...
                        run_w_star, run_average_stride_ftotal, ...
                        best_k, best_c, t_sim, run_tstride_avg, ...
                        num_mdl_pts, num_exp_pts, mse_npoints ...
                    );
[~, ~, ~, ~, ~, hop_w_star] = robustMinCOT('h', target_speed, target_freq, 2*best_k, 2*best_c, 2*best_I, 0);
[hop_t_plot, hop_mdl_forces, hop_exp_forces] = norm_and_plot_mdl_vs_exp( ...
                        hop_w_star, hop_average_stride_ftotal, ...
                        2*best_k, 2*best_c, t_sim, hop_tstride_avg, ...
                        num_mdl_pts, num_exp_pts, mse_npoints ...
                    );

figure;
subplot(1, 2, 1)
title("Best fitting plots (Run)");
hold on;
plot(run_t_plot, run_mdl_forces);
plot(run_t_plot, run_exp_forces);
xlabel("Time (secs)");
ylabel("Force (BWs)");
hold off;

subplot(1, 2, 2)
title("Best fitting plots (Hop)");
hold on;
plot(hop_t_plot, hop_mdl_forces);
plot(hop_t_plot, hop_exp_forces);
xlabel("Time (secs)");
ylabel("Force (BWs)");
hold off;

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);