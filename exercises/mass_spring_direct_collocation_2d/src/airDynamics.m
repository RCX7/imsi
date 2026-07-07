% airDynamics.m
% Roger Chen
% 2026-06-17
% Allows for input to control system in the air.

% groundDynamics.m
% Roger Chen
% 2026-06-17
% Defines the ode function for ode45 based on input length signals

function dy = airDynamics(t, y, w)
    [g, Lp, m, k, c, z0, zdot0, x0, xdot0,...
        tfinal, N, n_state_vars, dt, xdot_target] = constants();

    % interpolate w
    Tf = w(end - 1);
    if t < Tf
        N = numel(w) - 2;
        t_itrp = linspace(0, Tf, N);
        u = interp1(t_itrp, w(1:end-2), t);           % La_dot (control signal) at point t
    else
        u = 0;
    end

    dy = [ y(2);
           -g;
           y(4);
           0;
           u ];

end