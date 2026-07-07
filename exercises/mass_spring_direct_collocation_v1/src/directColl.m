% directColl.m
% Roger Chen
% 2026-18-06
% Runs direct collocation to optimize a bouncing actuated spring

clc; clearvars -except N; % close all;

[g, Lp, m, k, c, h0, hdot0, tfinal, new_N, n_state_vars, dt] = constants();
if new_N ~= N
    disp("generating new jacobians");
    generateJacobians;
    N = new_N;
end

A = [];
b = [];
Aeq = [];
beq = [];

ladot_min = -5;
la_min = 0.7;
lb = -inf(((n_state_vars + 1) * N + 1), 1);
lb(end) = 0;                               % no negative time
lb(1:N) = ladot_min;
lb(3*N+1:4*N) = la_min;

ladot_max = 5;
la_max = 0.9;
ub = inf(((n_state_vars + 1) * N + 1), 1);
ub(end) = tfinal;
ub(1:N) = ladot_max;
ub(3*N+1:4*N) = la_max;

%cost = @(w) sum(cumtrapz(w(1:N)));         % minimize change from original length
cost = @(w) 0;
%cost = @(w) trapz(w(1:N))^2;
cost = @costFun;

% GUESS AND RUN OPTIMIZATION
w0 = zeros(((n_state_vars + 1) * N + 1), 1);  % same knot points for each variable

% playing around with inital guesses for control (far more robust)
%w0(1:N) = 500;
%w0(1:2:N) = -500; 

w0(N + 1) = h0;
w0(2*N + 1) = hdot0;
% w0(2*N+1:2:3*N) = 1;
% w0(2*N+2:2:3*N) = -1;
w0(3*N + 1) = Lp;
w0(end) = tfinal;
options = optimoptions("fmincon", "Display", "iter", ...
                       "MaxFunctionEvaluations", 150000, ...  % Default is 3000
                       "MaxIterations", 10000,...
                       "SpecifyObjectiveGradient",true,...
                       "SpecifyConstraintGradient",true);

% 'EnableFeasibilityMode', true, ...
% 'SubproblemAlgorithm', 'cg'
w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub, @dc_nlc_spring, options);
optimal_step = w_star(1:N);
t_final_star = w_star(end);
t_plot = linspace(0, t_final_star, N);


subplot(4, 1, 1);
hold on;
scatter(t_plot, w_star(1:N));
plot(t_plot, w_star(1:N));
ylabel("control (ldot)");
xlabel("time");
ylim([ladot_min, ladot_max]);
hold off;

subplot(4, 1, 2);
hold on;
scatter(t_plot, w_star(N+1:2*N));
plot(t_plot, w_star(N+1:2*N));
hold off;
ylabel("position");
xlabel("time");

subplot(4, 1, 3);
hold on;
scatter(t_plot, w_star(2*N+1:3*N));
plot(t_plot, w_star(2*N+1:3*N));
hold off;
ylabel("velocity");
xlabel("time");

subplot(4, 1, 4);
hold on;
scatter(t_plot, w_star(3*N+1:4*N));
plot(t_plot, w_star(3*N+1:4*N));
ylim([la_min, la_max]);
ylabel("la");
xlabel("time");
hold off;
