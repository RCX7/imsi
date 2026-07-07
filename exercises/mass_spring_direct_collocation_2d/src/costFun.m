% costFun.m
% Roger Chen
% 2026-06-21
% A cost function based on metabolic work (integral( force * dla ))

function [cost, jcost] = costFun(w)
    [g, Lp, m, k, c, z0, zdot0, x0, xdot0,...
        tfinal, N, n_state_vars, dt, xdot_target] = constants();
    
    % precise abs value approx.
    eps = 0.001;                                         % abs smoothing param
    smAbs = @(x) sqrt(x.^2 + eps^2) - eps;
    %

    %cost = 0;
    toe_pos = w(end-3);
    zdists = w(2*N+1:3*N);
    xdists = w(8*N+1:9*N) - toe_pos;
    dists = sqrt(zdists.^2 + xdists.^2);
    dzdists = w(5*N+1:6*N);
    dxdists = w(11*N+1:12*N);
    ddists = (xdists.*dxdists + zdists.*dzdists) ./ dists;
    us = w(1:N);
    las = w(13*N+1:14*N);
    forces = k * (las - dists) + c * (us - ddists);
    work = us .* forces;
    unsigned_work = smAbs(work);
    % trapezoidal integration over time period to find cost
    cost = trapz(w(end-2), unsigned_work);
    xdist_travelled = w(10*N) - w(7*N+1);
    cost = cost / (m * g * xdist_travelled);

    if ~isa(w, "sym")
        jcost = Jcost(w);
    else
        jcost = 0;
end
