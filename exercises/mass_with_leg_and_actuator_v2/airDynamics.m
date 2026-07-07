% airDynamics.m
% Roger Chen
% 2026-06-17
% Allows for input to control system in the air.

% groundDynamics.m
% Roger Chen
% 2026-06-17
% Defines the ode function for ode45 based on input length signals

function dx = airDynamics(t, y, w)
    [g, la0, m, k, c, h0, hdot0] = constants();

    % interpolate w
    Tf = w(end);
    if Tf > t
        N = numel(w) - 1;
        t_itrp = linspace(0, Tf, N);
        u = interp1(t_itrp, w(1:end-1), t);           % La_dot (control signal) at point t
    else
        u = 0;
    end

    dx = [ y(2);
           -g;
           u];

end