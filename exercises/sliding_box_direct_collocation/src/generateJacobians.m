% generateJacobians.m
% Roger Chen
% 2026-06-21
% used to generate the jacobians to input gradients to fmincon
% jacobians will vary with N, cost function, and nlc

[grav, m, s_final, N, num_state_vars] = constants();
xsym = sym("x", [(num_state_vars + 1) * N + 1, 1]);

JCost = jacobian(cost(xsym), xsym);

[C, Ceq, ~, ~] = nlc_sb(xsym);
JC = jacobian(C, xsym);
JCeq = jacobian(Ceq, xsym);

matlabFunction(JCost, "File", "JCost");
matlabFunction(JC, "File", "JC");
matlabFunction(JCeq, "File", "JCeq", "Vars", {xsym});