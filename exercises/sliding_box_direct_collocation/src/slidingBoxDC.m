% slidingBoxDC.m
% Roger Chen
% 2026-19-06
% Simplest example of direct collocation

clc; clear; close all;

[grav, m, s_final, N, num_state_vars] = constants();
A = [];
b = [];
Aeq = [];
beq = [];

lb = -inf(((num_state_vars + 1) * N) + 1, 1);
lb(end) = 0;                                    % no time travelling
fmin = -5;
lb(1:N) = fmin;

ub = inf(((num_state_vars + 1) * N) + 1, 1);
fmax = 5;
ub(1:N) = fmax;

cost = @(w) w(end);
w0 = ones(((num_state_vars + 1) * N) + 1, 1);

options = optimoptions("fmincon", "MaxFunctionEvaluations",100000,...
                        "Display", "Iter",...
                        "SpecifyConstraintGradient",true);
                        %"SpecifyObjectiveGradient",true);
                        
tic;
w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub, @nlc_sb, options);
toc;

tf = w_star(end);
t_plot = linspace(0, tf, N);

subplot(3, 1, 1);
hold on;
plot(t_plot, w_star(1:N));
scatter(t_plot, w_star(1:N));
xlabel("time (s)");
ylabel("force");
hold off;

subplot(3, 1, 2);
hold on;
plot(t_plot, w_star(N+1:2*N));
scatter(t_plot, w_star(N+1:2*N));
xlabel("time (s)");
ylabel("position");
hold off;

subplot(3, 1, 3);
hold on;
plot(t_plot, w_star(2*N+1:3*N));
scatter(t_plot, w_star(2*N+1:3*N));
xlabel("time (s)");
ylabel("velocity");
hold off;