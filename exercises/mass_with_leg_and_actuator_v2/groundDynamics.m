% groundDynamics.m
% Roger Chen
% 2026-06-17
% Defines the ode function for ode45 based on input length signals

function dx = groundDynamics(t, y, w)
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

    % Compute force.
    F = k * (y(3) - y(1)) + c * (u - y(2));       % Based on this definition y(3) is the length of the spring
    dx = [ y(2);
           (F / m) - g;
           u];

end