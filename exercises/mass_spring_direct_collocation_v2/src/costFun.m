% costFun.m
% Roger Chen
% 2026-06-21
% A cost function based on metabolic work (integral( force * dla ))

function [cost, jcost] = costFun(w)
    [g, Lp, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants();
    
    eps = 0.001;                                          % abs smoothing param
    smAbs = @(x) sqrt(x^2 + eps^2) - eps;

    % k = 20;                                              % abs smoothing param
    % smAbs = @(x) (log(cosh(k * x))) / k;
    
    cost = 0;
    for iter=1:N                                         % for each knot point in the state (except the last one)
           vel = w(5*N + iter);
           pos = w(2*N + iter);
           la = w(7*N + iter);
           u = w(iter);

           % contact_factor = 0.5 * (1 + tanh(tanh_smooth * (Lp - pos)));   % like a smooth reLU
           force = (k * (la - pos) + c * (u - vel));
           work = smAbs(force * u);
           cost = cost + work;
    end

    if ~isa(w, "sym")
        jcost = Jcost(w);
    else
        jcost = 0;
end
