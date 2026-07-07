% directColl.m
% Roger Chen
% 2026-18-06
% Runs direct collocation to optimize a bouncing actuated spring

clc; clearvars -except N w_star jcost_res jcost_mask; close all;

opt = true;

%% PREPARING OPTIMIZATION
[g, Lp, m, k, c, z0, zdot0, x0, xdot0,...
    tfinal, new_N, n_state_vars, dt, xdot_target] = constants();
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
lb = -inf(((n_state_vars + 1) * N + 4), 1);
lb(end-3) = 0;
lb(end-2:end) = N * 1e-3;                        % no negative time
lb(1:N) = ladot_min;
lb(13*N+1:14*N) = la_min;

ladot_max = 5;
la_max = 0.9;
ub = inf(((n_state_vars + 1) * N + 4), 1);
ub(end-3) = Lp;                                  % position of toe
ub(end-2:end) = tfinal;
ub(end-1) = 0.2;
ub(1:N) = ladot_max;
ub(13*N+1:14*N) = la_max;

%cost = @(w) 0;                 % no cost function
%cost = @(w) w(end-1);         % contact time
cost = @costFun;              % mechanical work

%% GUESS AND RUN OPTIMIZATION
%w0 = linspace(0, 1, ((n_state_vars + 1) * N + 4)).';  % same knot points for each variable
w0 = zeros((n_state_vars + 1) * N + 4, 1) + 0.5;

% playing around with inital guesses for control (far more robust)
%w0(1:N) = 500;
%w0(1:2:N) = -500; 
w0(1:N) = linspace(0, ladot_max, N);
w0(N + 1) = z0;
w0(5*N + 1) = zdot0;
w0(8*N + 1) = x0;
w0(10*N) = 3;           % arbitrary nonzero value to keep from undefined initial cost function
w0(11*N + 1) = xdot0;
% w0(2*N+1:2:3*N) = 1;
% w0(2*N+2:2:3*N) = -1;
w0(13*N + 1) = Lp;
w0(end-3) = 0.15;
w0(end - 2) = tfinal / 3;
w0(end-1) = tfinal / 3;
w0(end) = tfinal / 3;
options = optimoptions("fmincon", "Display", "iter", ...
                       "MaxFunctionEvaluations", 5000, ...  % Default is 3000
                       "SpecifyObjectiveGradient",true,...
                       "SpecifyConstraintGradient",true,...
                       'ConstraintTolerance', 5e-6,...
                       'StepTolerance', 1e-10,...
                       'OutputFcn', @(w, optimValues, state) saveCurrentW(w, optimValues, state),...
                       "MaxIterations", 1000);
%"CheckGradients", true,...               % for debugging
%'FiniteDifferenceType', 'central',...    % passes initial gradient check

% 'EnableFeasibilityMode', true, ...
% 'SubproblemAlgorithm', 'cg'
if opt
    disp("starting opt");
    checkGradients(cost, w0, options);
    w_star = fmincon(cost, w0, A, b, Aeq, beq, lb, ub, @dc_nlc_spring, options);
end
optimal_step = w_star(1:N);

flight1_pad = zeros(round(w_star(end-2) / w_star(end-1) * N), 1);
flight2_pad = zeros(round((w_star(end) / w_star(end-1)) * N), 1);
optimal_step = [flight1_pad; optimal_step; flight2_pad];

tf1_end = w_star(end-2);
ts_end = tf1_end + w_star(end - 1);
tf2_end = ts_end + w_star(end);
step_time = tf2_end;                        % used in ode45 sim
toe_pos = w_star(end-3);

tflight1 = linspace(0, tf1_end, N);
tstance = linspace(tf1_end,  ts_end, N);
tflight2 = linspace(ts_end, tf2_end, N);


%% PLOTTING
% position (for plotting)
zf1 = w_star(N+1:2*N);
zs = w_star(2*N+1:3*N);
zf2 = w_star(3*N+1:4*N);
xf1 = w_star(7*N+1:8*N);
xs = w_star(8*N+1:9*N);
xf2 = w_star(9*N+1:10*N);

% velocities (for plotting)
dzf1 = w_star(4*N+1:5*N);
dzs = w_star(5*N+1:6*N);
dzf2 = w_star(6*N+1:7*N);
dxf1 = w_star(10*N+1:11*N);
dxs = w_star(11*N+1:12*N);
dxf2 = w_star(12*N+1:13*N);


subplot(2, 2, 1);
hold on;
scatter(tstance, w_star(1:N));
plot(tstance, w_star(1:N));
hold off;
ylabel("control (ldot)");
xlabel("time");
xlim padded;
ylim([ladot_min, ladot_max]);


subplot(2, 2, 3);
hold on;
yline(0, 'LineWidth',3);
scatter(xf1, zf1, 'b');
plot(xf1, zf1, 'b-');
scatter(xs, zs, 'r');
plot(xs, zs, 'r-');
scatter(xf2, zf2, 'c');
plot(xf2, zf2, 'c-');

plot(xs, w_star(13*N+1:14*N), 'r--');
plot(xf1(end) + w_star(end-3), 0, 'x', 'MarkerSize', 10, 'MarkerFaceColor', 'black');
plot([w_star(8*N), w_star(8*N) + w_star(end-3)], [w_star(2*N), 0], "k-", LineWidth=1);                 % plot leg at heel strike
hold off;
xlim padded;
ylim padded;
axis equal;
ylabel("z position");
xlabel("x position");


subplot(2, 2, 4);
hold on;
scatter(dxf1, dzf1, 'b');
plot(dxf1, dzf1, 'b-');
scatter(dxs, dzs, 'r');
plot(dxs, dzs, 'r-');
scatter(dxf2, dzf2, 'c');
plot(dxf2, dzf2, 'c-');
hold off;
xlim padded;
ylim padded;
axis equal;
ylabel("z velocity");
xlabel("x velocity");


subplot(2, 2, 2);
hold on;
scatter(tstance, w_star(13*N+1:14*N));
plot(tstance, w_star(13*N+1:14*N));
hold off;
xlim padded;
ylim([la_min, la_max]);
ylabel("la");
xlabel("time");

