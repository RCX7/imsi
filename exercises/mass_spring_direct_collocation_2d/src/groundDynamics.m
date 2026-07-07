% groundDynamics.m
% Roger Chen
% 2026-06-17
% Defines the ode function for ode45 based on input length signals

function dx = groundDynamics(t, y, w, init_x, step_period)
    [g, Lp, m, k, c, z0, zdot0, x0, xdot0,...
        tfinal, N, n_state_vars, dt, xdot_target] = constants();

    % interpolate w
    residual = floor(t / step_period) * -0.001;             % small manually tuned offset
    t = mod(t, step_period) + residual;                    % time in step
    t = t - w(end-2);
    Tf = w(end-1);
    if Tf > t
        N = numel(w) - 2;
        t_itrp = linspace(0, Tf, N);
        u = interp1(t_itrp, w(1:end-2), t);           % La_dot (control signal) at point t
    else
        u = 0;
    end

    % Compute force.
    toe_pos = init_x + w(end);
    xdist = y(3) - toe_pos;
    zdist = y(1);
    dist = sqrt(xdist^2 + zdist^2);
    dxdist = y(4); dzdist = y(2);
    ddist = (xdist*dxdist + zdist*dzdist) / dist;
    F = k * (y(5) - dist) + c * (u - ddist);       % Based on this definition y(3) is the length of the spring
    Fz = F * (zdist / dist);
    Fx = F * (xdist / dist);
    dx = [ y(2);
           (Fz / m) - g;
           y(4);
           (Fx / m);
           u];

end