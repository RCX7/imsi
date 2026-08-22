% norm_and_plot_mdl_vs_exp.m
% Roger Chen
% 2026-08-10
% Normalize experimental data and plot it against model results
% Extremely verbose title. Oops.


function [t_common, mdl_forces, exp_forces] = norm_and_plot_mdl_vs_exp(w_star, average_stride_ftotal, ...
                                    k, c, t_sim, tstride_avg, ...
                                    num_mdl_pts, num_exp_pts, mse_npoints)
    [control, states, addDecs] = decToMats(w_star);
    mdl_forces = computeForces(states, control, k, c);
    
    %%  interpolate to same timings
    % mdl_tstance = addDecs(1) * t_sim;
    % exp_tstance = tstride_avg;               % loaded from mean force .mat file
    % tmax = max([mdl_tstance, exp_tstance]);  % find which time is longer
    % 
    % mdl_forces = interp1(linspace(0, mdl_tstance, num_mdl_pts), mdl_forces, linspace(5e-5, tmax, mse_npoints));
    % exp_forces = interp1(linspace(0, exp_tstance, num_exp_pts), average_stride_ftotal, linspace(5e-5, tmax, mse_npoints));

    mdl_tstance = addDecs(1) * t_sim;
    exp_tstance = tstride_avg;               % loaded from mean force .mat file
    tmax = max([mdl_tstance, exp_tstance]);  % find which time is longer

    t_common = linspace(0, tmax, mse_npoints);
    
    % Interpolate using the common time vector, adding 'linear' (or 'spline') and 
    % handling potential extrapolation safely if bounds differ slightly
    mdl_forces = interp1(linspace(0, mdl_tstance, num_mdl_pts), mdl_forces, t_common, 'linear', 'extrap');
    exp_forces = interp1(linspace(0, exp_tstance, num_exp_pts), average_stride_ftotal, t_common, 'linear', 'extrap');

    mdl_forces = clip(mdl_forces, 0, inf);
    exp_forces = clip(exp_forces, 0, inf);

    mdl_forces(isnan(mdl_forces)) = 0;
    exp_forces(isnan(exp_forces)) = 0;
end