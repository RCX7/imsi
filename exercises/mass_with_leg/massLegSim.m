% massLegSim
% Roger Chen
% 2026-06-15

clc
clear
close all

% Define variables
simulation = true;
g = 9.81;           % Gravity (m/s^2)
L = 0.8;            % Leg length (meters)
m = 0.2;              % Mass (kg)
k = 100;            % Spring constant (N/m)
tfinal = 5;        % Time (seconds)

h0 = 1.5;          % Starting height (m)
hdot0 = -1;          % Starting velocity (m/s)

% Define dynamics equations
ground_dynamics = @(t, y) [ y(2); ((L - y(1)) * (k/m) - g) ]; % when on ground
air_dynamics = @(t, y)  [ y(2) ; -g];                         % when in air

% Run simulation with ODE45
%t_vec = [];
%h_vec = [];

% determine initial state:
if L >= h0
    % STATE IS GROUND
    options = odeset('Events', @leaveGroundEvent);
    [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
        [0 tfinal], [h0; hdot0], options);
    state = "air";  % done with ground time
else
    % STATE IS AIR
    options = odeset('Events', @contactGroundEvent);
    [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
        [0 tfinal], [h0; hdot0], options);
    state = "ground";    % done with air time
end
disp(state);

t_vec = t_v;
h_vec = h_v;

while t_vec(end) <= tfinal
    
    if isempty(h_e)
        break;
    end
    
    if state == "air"
        options = odeset('Events', @contactGroundEvent);
        [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
            [t_e tfinal], [h_e(1); h_e(2)], options);
        state = "ground";
    else
        options = odeset('Events', @leaveGroundEvent);
        [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
            [t_e tfinal], [h_e(1); h_e(2)], options);
        state = "air";
    end
    t_vec = [t_vec; t_v(2:end, :)];
    h_vec = [h_vec; h_v(2:end, :)];
end

subplot(2, 1, 1);
hold on;
plot(t_vec, h_vec(:,1), 'r-');
yline(L - ((m * g) / k), 'r--');        % plot equilibrium condition
ylabel("vertical displacement (m)");
xlabel("time (sec)");

subplot(2, 1, 2);
plot(t_vec, h_vec(:,2), 'b--');
ylabel("vertical velocity (m/sec)");
xlabel("time (sec)");

if simulation
    pause(1);
    close all;
    figure
    
    FPS = 20;                                       % defining framerate
    t_anim = 1:1/FPS:t_vec(end);                    % defining framerate
    h_anim = interp1(t_vec, h_vec(:, 1), t_anim);   % finding points at these frames
    
    for iter = 1:numel(t_anim)
        plot([0 0], [max(0, (h_anim(iter) - L)) h_anim(iter)], ...
            'k--', 'LineWidth', k * 0.02);
        hold on;
        axis equal;
        axis([-1 1 -0.25 5]);
        
        yline(0, 'k-');
        plot(0, h_anim(iter), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
        drawnow;
        hold off;
    end
end

