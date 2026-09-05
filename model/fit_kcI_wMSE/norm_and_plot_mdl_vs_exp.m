% norm_and_plot_mdl_vs_exp.m
% Roger Chen
% 2026-08-10
% Normalize experimental data and plot it against model results
% Extremely verbose title. Oops.


function [t_common, mdl_forces_itrp, exp_forces_itrp] = ...
           norm_and_plot_mdl_vs_exp(mdl_forces, exp_forces, ...
                                    mdl_tstance_norm, mdl_tflight_norm, exp_tstance, exp_tflight, ...
                                    num_mdl_pts, num_exp_pts, mse_npoints)
    % MODEL TIMES SHOULD BE NORMALIZED USING t_sim PRIOR TO INPUT
    
    %%  interpolate to same timings for two strides
    mdl_tstride_norm = mdl_tstance_norm + mdl_tflight_norm;
    exp_tstride_norm = exp_tstance + exp_tflight;
    tmax = 2 * max([mdl_tstride_norm, exp_tstride_norm]);  % find which time is longer
    t_common = linspace(0, tmax, mse_npoints);

    mdl_tstance_itrp = linspace(0, mdl_tstance_norm, num_mdl_pts);
    exp_tstance_itrp = linspace(0, exp_tstance, num_exp_pts);
    
    mdl_forces_itrp = interp1(mdl_tstance_itrp, mdl_forces, 0:0.005:mdl_tstride_norm, 'linear');
    exp_forces_itrp = interp1(exp_tstance_itrp, exp_forces, 0:0.005:exp_tstride_norm, 'linear');
    mdl_forces_itrp = [mdl_forces_itrp, mdl_forces_itrp];
    exp_forces_itrp = [exp_forces_itrp, exp_forces_itrp];

    t_mdl = linspace(0, 2 * mdl_tstride_norm, numel(mdl_forces_itrp));
    t_exp = linspace(0, 2 * exp_tstride_norm, numel(exp_forces_itrp));

    mdl_forces_itrp = interp1(t_mdl, mdl_forces_itrp, t_common, 'linear');
    exp_forces_itrp = interp1(t_exp, exp_forces_itrp, t_common, 'linear');

    % CLIP NEGATIVES AND FILL NaN WITH 0s
    mdl_forces_itrp(isnan(mdl_forces_itrp)) = 0;
    exp_forces_itrp(isnan(exp_forces_itrp)) = 0;

    mdl_forces_itrp = clip(mdl_forces_itrp, 0, inf);
    exp_forces_itrp = clip(exp_forces_itrp, 0, inf);

    % let return time be stride percentage
    gait_pct = linspace(0, 100, mse_npoints);
end