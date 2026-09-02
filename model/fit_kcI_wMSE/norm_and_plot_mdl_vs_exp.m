% norm_and_plot_mdl_vs_exp.m
% Roger Chen
% 2026-08-10
% Normalize experimental data and plot it against model results
% Extremely verbose title. Oops.


function [t_common, mdl_forces_itrp, exp_forces] = norm_and_plot_mdl_vs_exp(mdl_forces, average_stride_ftotal, ...
                                    t_sim, tstride_avg, ...
                                    num_mdl_pts, num_exp_pts, mse_npoints)
    % [control, states, addDecs] = decToMats(w_star);
    % mdl_forces = computeForces(states, control, k, c);
    
    %%  interpolate to same timings
    mdl_tstance = addDecs(1) * t_sim;
    exp_tstance = tstride_avg;               % loaded from mean force .mat file
    tmax = max([mdl_tstance, exp_tstance]);  % find which time is longer

    t_common = linspace(0, tmax, mse_npoints);
    
    % Interpolate using the common time vector, adding 'linear' (or 'spline') and 
    % handling potential extrapolation safely if bounds differ slightly
    mdl_forces_itrp = interp1(linspace(0, mdl_tstance, num_mdl_pts), mdl_forces, t_common, 'linear', 'extrap');
    exp_forces = interp1(linspace(0, exp_tstance, num_exp_pts), average_stride_ftotal, t_common, 'linear', 'extrap');

    mdl_forces_itrp = clip(mdl_forces_itrp, 0, inf);
    exp_forces = clip(exp_forces, 0, inf);

    mdl_forces_itrp(isnan(mdl_forces_itrp)) = 0;
    exp_forces(isnan(exp_forces)) = 0;
end