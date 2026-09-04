% norm_and_plot_mdl_vs_exp.m
% Roger Chen
% 2026-08-10
% Normalize experimental data and plot it against model results
% Extremely verbose title. Oops.


function [gait_pct, mdl_forces_itrp, exp_forces] = ...
           norm_and_plot_mdl_vs_exp(mdl_forces, exp_forces, ...
                                    mdl_tstance_norm, mdl_tflight_norm, exp_tstance, exp_tflight, ...
                                    num_mdl_pts, num_exp_pts, mse_npoints)
    % MODEL TIMES SHOULD BE NORMALIZED USING t_sim PRIOR TO INPUT
    % should handle running differently no?
    
    %%  interpolate to same timings
    mdl_tstride_norm = mdl_tstance_norm + mdl_tflight_norm;
    exp_tstride_norm = exp_tstance + exp_tflight;
    tmax = max([mdl_tstride_norm, exp_tstride_norm]);  % find which time is longer
    t_common = linspace(0, tmax, mse_npoints);
    
    mdl_tstance_itrp = (mdl_tstance_norm / mdl_tstride_norm) * tmax;
    exp_tstance_itrp = (exp_tstance / exp_tstride_norm) * tmax;
    % Interpolate using the common time vector, adding 'linear' (or 'spline') and 
    % handling potential extrapolation safely if bounds differ slightly
    mdl_forces_itrp = interp1(linspace(0, mdl_tstance_itrp, num_mdl_pts), mdl_forces, t_common, 'linear', 'extrap');
    exp_forces = interp1(linspace(0, exp_tstance_itrp, num_exp_pts), exp_forces, t_common, 'linear', 'extrap');
    
    % CLIP NEGATIVES AND FILL WITH 0s
    mdl_forces_itrp = clip(mdl_forces_itrp, 0, inf);
    exp_forces = clip(exp_forces, 0, inf);

    mdl_forces_itrp(isnan(mdl_forces_itrp)) = 0;
    exp_forces(isnan(exp_forces)) = 0;

    % let return time be stride percentage
    gait_pct = linspace(0, 100, mse_npoints);
end