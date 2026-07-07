% directColl.m
% Roger Chen
% 2026-18-06
% Runs direct collocation to optimize a bouncing actuated spring

clc; clearvars -except N; close all;

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
lb = -inf(((n_state_vars + 1) * N + 3), 1);
lb(end-2:end) = 0;                               % no negative time
lb(1:N) = ladot_min;
lb(7*N+1:8*N) = la_min;

ladot_max = 5;
la_max = 0.9;
ub = inf(((n_state_vars + 1) * N + 3), 1);
ub(end-2:end) = tfinal;
ub(end-1) = 0.2;
ub(1:N) = ladot_max;
ub(7*N+1:8*N) = la_max;

%cost = @(w) sum(cumtrapz(w(1:N)));         % minimize change from original length
%cost = @(w) 0;
%cost = @(w) w(end-1);
cost = @costFun;

% GUESS AND RUN OPTIMIZATION
w0 = zeros(((n_state_vars + 1) * N + 3), 1) + 0.2;  % same knot points for each variable

% playing around with inital guesses for control (far more robust)
%w0(1:N) = 500;
%w0(1:2:N) = -500; 

w0(N + 1) = h0;
w0(2*N + 1) = hdot0;
% w0(2*N+1:2:3*N) = 1;
% w0(2*N+2:2:3*N) = -1;
w0(3*N + 1) = Lp;
w0(end - 2) = tfinal / 3;
w0(end-1) = tfinal / 3;
w0(end) = tfinal / 3;
options = optimoptions("fmincon", "Display", "iter", ...
                       "MaxFunctionEvaluations", 150000, ...  % Default is 3000
                       "MaxIterations", 10000,... 
                       "SpecifyObjectiveGradient",true,...
                       "SpecifyConstraintGradient",true);

% 'EnableFeasibilityMode', true, ...
% 'SubproblemAlgorithm', 'cg'
w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub, @dc_nlc_spring, options);
optimal_step = w_star(1:N);

flight1_pad = zeros(round(w_star(end-2) / w_star(end-1) * N), 1);
flight2_pad = zeros(round((w_star(end) / w_star(end-1)) * N), 1);
optimal_step = [flight1_pad; optimal_step; flight2_pad];

tf1_end = w_star(end-2);
ts_end = tf1_end + w_star(end - 1);
tf2_end = ts_end + w_star(end);
step_time = tf2_end;                        % used in ode45 sim

tflight1 = linspace(0, tf1_end, N);
tstance = linspace(tf1_end,  ts_end, N);
tflight2 = linspace(ts_end, tf2_end, N);

subplot(4, 1, 1);
hold on;
scatter(tstance, w_star(1:N));
plot(tstance, w_star(1:N));
hold off;
ylabel("control (ldot)");
xlabel("time");
xlim([0, tf2_end]);
ylim([ladot_min, ladot_max]);


subplot(4, 1, 2);
hold on;
scatter(tflight1, w_star(N+1:2*N), 'b');
plot(tflight1, w_star(N+1:2*N), 'b-');
scatter(tstance, w_star(2*N+1:3*N), 'r');
plot(tstance, w_star(2*N+1:3*N), 'r-');
scatter(tflight2, w_star(3*N+1:4*N), 'b');
plot(tflight2, w_star(3*N+1:4*N), 'b-');
hold off;
xlim([0, tf2_end]);
ylabel("position");
xlabel("time");


subplot(4, 1, 3);
hold on;
scatter(tflight1, w_star(4*N+1:5*N), 'b');
plot(tflight1, w_star(4*N+1:5*N), 'b-');
scatter(tstance, w_star(5*N+1:6*N), 'r');
plot(tstance, w_star(5*N+1:6*N), 'r-');
scatter(tflight2, w_star(6*N+1:7*N), 'b');
plot(tflight2, w_star(6*N+1:7*N), 'b-');
hold off;
xlim([0, tf2_end]);
ylabel("velocity");
xlabel("time");


subplot(4, 1, 4);
hold on;
scatter(tstance, w_star(7*N+1:8*N));
plot(tstance, w_star(7*N+1:8*N));
hold off;
xlim([0, tf2_end]);
ylim([la_min, la_max]);
ylabel("la");
xlabel("time");

