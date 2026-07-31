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
trials = {'hop_1ms_4min'; 'run_1ms_4min'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
mse_npoints = 35;

[num_mdl_pts, ~] = simConstants();
[~, ~, ~, ~, ~, ~, target_speed, target_freq, I, t_sim, l_uN] =...
    physConstants(gait);

ks = 40:2.5:70;
cs = 0.3:0.05:0.8;
[k_mesh, c_mesh] = meshgrid(ks, cs);
k_vec = k_mesh(:);
c_vec = c_mesh(:);


load("avgForceCurves/" + curr_trial + "_ftotal_curve");  % called "average_stride_ftotal"
m = 92.65; g = 9.81;
average_stride_ftotal = average_stride_ftotal / (m * g);

rmpath(singleOpt_path);

%% set up output (flattened for parallelization)
n_sims = numel(k_vec);
mse_landscape = zeros(n_sims, 1);

%% parallelize optimizations
addpath(multiOpt_path);
% parfor i=1:n_sims
for j=1:6
    i = j * 15;
    k = k_vec(i);
    c = c_vec(i);
    
    [~, ~, ~, ~, ~, w_star] = ...
                robustMinCOT(gait, target_speed, target_freq, k, c, I, 0);
    if w_star
        [control, states, addDecs] = decToMats(w_star);
        mdl_forces = computeForces(states, control, k, c);
        
        %%  interpolate to same timings
        mdl_tstance = addDecs(1);
        exp_tstance = numel(average_stride_ftotal) * 5e-4;
        tmax = max([mdl_tstance, exp_tstance]);  % find which time is longer
        
        mdl_forces = interp1(linspace(5e-5, mdl_tstance, num_mdl_pts), mdl_forces, linspace(5e-5, tmax, mse_npoints));
        exp_forces = interp1(5e-4:5e-4:exp_tstance, average_stride_ftotal, linspace(5e-5, tmax, mse_npoints));
        mdl_forces(isnan(mdl_forces)) = 0;
        exp_forces(isnan(exp_forces)) = 0;
    
        %% measure MSE and store
        mse = fcurve_mse(mdl_forces, exp_forces);
        mse_landscape(i) = mse;
    end
    subplot(2, 3, j);
    title("Index: " + i + " k: " + k + " c: " + c + " mse: " + mse);
    hold on;
    plot(mdl_forces);
    plot(exp_forces);
    hold off;
% end
end

mse_landscape = reshape(mse_landscape, size(k_mesh));
mse_landscape(mse_landscape == 0) = max(mse_landscape, [], "all");
% surf(ks, cs, mse_landscape);

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);
