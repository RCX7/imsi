% costFun.m
% Roger Chen
% 2026-06-21
% A cost function based on metabolic work (integral( force * dla ))

function [cost, jcost] = costFun(w)
    [g, Lp, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants();
    
    tanh_smooth = 1000;                                % controls smoothness of hyperbolic tangent
    k = 20;                                          % abs smoothing param
    % smAbs = @(x) ((2/k) * log(1 + exp(k * x))) - x - ((2/k) * log(2)); % smooth absolute value
    smAbs = @(x) (log(cosh(k * x))) / k;

    cost = 0;
    for iter=1:N                                         % for each knot point in the state (except the last one)
           vel = w(2*N + iter);
           pos = w(N + iter);
           la = w(3*N + iter);
           u = w(iter);

           contact_factor = 0.5 * (1 + tanh(tanh_smooth * (Lp - pos)));   % like a smooth reLU
           force = contact_factor * (k * (la - pos) + c * (u - vel));
           % disp('-------');
           % disp(force * u);
           work = smAbs(force * u);
           % disp(work);
           % disp('-------');
           cost = cost + work;
    end

    if ~isa(w, "sym")
        jcost = Jcost(w);
    else
        jcost = 0;
end
