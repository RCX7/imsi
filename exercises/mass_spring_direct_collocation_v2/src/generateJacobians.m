% generateJacobians.m
% Roger Chen
% 2026-06-21
% generates the jacobian matricies for faster optimization

[g, Lp, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants();

xsym = sym("x", [(n_state_vars + 1) * N  + 3, 1]);

[cost, ~] = costFun(xsym);
Jcost = jacobian(cost, xsym);
matlabFunction(Jcost, "File","Jcost", "Vars", {xsym});

[c, ceq, ~, ~] = dc_nlc_spring(xsym);
JC = jacobian(c, xsym);
JCeq = jacobian(ceq, xsym);

matlabFunction(JC, "File","JC");
matlabFunction(JCeq, "File","JCeq", "Vars", {xsym});