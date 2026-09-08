% fit_kc_parRunHop.m
% Roger Chen
% 2026-09-07
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves. OPERATES UNDER THE ASSUMPTION / CONSTRAINT THAT
% HOPPING HAS 2x INERTIA ONLY, leaving k and c free

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
gaits = ['r', 'h'];

subject = "S05";
m = 91.35; g = 9.81;

mse_npoints = 150;
[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, ~, ~, t_sim, l_uN] =...
    physConstants('r');  % actually does not depend on gait at all

%% settings and testing conditions
for gait_idx=1:2
    gait = gaits(gait_idx);
    
    % trials = {'hop_1ms_4min'; 'run_1ms_4min'};
    % trials = {'hop_1ms_unc_30sec'; 'run_1ms_unc_1min'};
    trials = {'hop_2ms'; 'run_2ms'};
    
    if gait == 'h', curr_trial = trials{1}; else, curr_trial = trials{2}; end
    % define variables
    switch gait
        case 'r'
            % ks = 40:2.5:70; for constrained version
            ks = 10:2.5:30;
            Is = 0.005:0.005:0.05;
            cs = 0.025:0.025:0.4;

            run_ks = ks;
            run_cs = cs;
            run_Is = Is;
        case 'h'
            ks = 25:1.25:50;
            Is = 0.01:0.01:0.1;   % twice Is
            cs = 0.05:0.025:0.8;

            hop_ks = ks;
            hop_cs = cs;
    end
    
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
        
        [~, ~, ~, ~, ~, w_star, ~] = ...
                    robustMinCOT(gait, target_speed, 0, k, c, I, 0);
    
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
    switch gait
        case 'r'
            run_mse_landscape = mse_landscape;
        case 'h'
            hop_mse_landscape = mse_landscape;
    end
end


% magic is in the search
combined_mses = zeros(1, numel(run_Is));
run_params = zeros(2, numel(run_Is));
hop_params = zeros(2, numel(run_Is));
for I_idx=1:numel(run_Is)
    curr_run_slice = run_mse_landscape(:, :, I_idx);
    curr_hop_slice = hop_mse_landscape(:, :, I_idx);

    [run_mse_min, run_lin_idx] = min(curr_run_slice, [], 'all', 'linear');
    [run_c_idx, run_k_idx] = ind2sub(size(curr_run_slice), run_lin_idx);
    run_min_k = run_ks(run_k_idx);
    run_min_c = run_cs(run_c_idx);
    disp(run_min_c); disp(run_min_k);
    
    [hop_mse_min, hop_lin_idx] = min(curr_hop_slice, [], 'all', 'linear');
    [hop_c_idx, hop_k_idx] = ind2sub(size(curr_hop_slice), hop_lin_idx);
    hop_min_k = hop_ks(hop_k_idx);
    hop_min_c = hop_cs(hop_c_idx);
    
    combined_mses(I_idx) = run_mse_min + hop_mse_min;
    run_params(:, I_idx) = [run_min_k; run_min_c];
    hop_params(:, I_idx) = [hop_min_k; hop_min_c];
end

[globalCombMinMSE, globalMinIdx] = min(combined_mses);
best_fit_run_params = run_params(:, globalMinIdx);
best_fit_hop_params = hop_params(:, globalMinIdx);

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);
