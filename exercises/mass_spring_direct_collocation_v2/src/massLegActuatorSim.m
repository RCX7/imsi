% massLegActuatorSim
% Roger Chen
% 2026-06-15
% Adapted from massLegSim code
% Adds a variable actuator length to the model.

clc
clearvars -except optimal_step step_time
close all

%% Define variables
simulation = false;
save_video = false;
video_name = "simulation_videos/optimal_step.mp4";

[g, la0, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants();
La_dot = [0 0 3];                     % Active length (Controlled by actuator) (same as u or w)
La_dot = [optimal_step.', step_time];
%La_dot = [optimal_step.', optimal_step(3:end).',0,0, optimal_step.', step_time * 3];
% La_dot = [optimal_step.', optimal_step(3:end).',0,0, optimal_step.', optimal_step.', optimal_step(3:end).',0,0, optimal_step.', 3];

%% Define dynamics equations
ground_dynamics = @(t, y) groundDynamics(t, y, La_dot);
air_dynamics = @(t, y)  airDynamics(t, y, La_dot);
leaveGroundEventHandler = @(t, y) leaveGroundEvent(t, y, La_dot);

%% Run simulation with ODE45 %%

% determine initial state:
if la0 >= h0
    % STATE IS GROUND
    options = odeset('Events', leaveGroundEventHandler);
    [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
        [0 tfinal], [h0; hdot0; la0], options);
    state = "air";      % done with ground time
else
    % STATE IS AIR
    options = odeset('Events', @contactGroundEvent);
    [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
        [0 tfinal], [h0; hdot0; la0], options);
    state = "ground";    % done with air time
end
disp(state);

t_vec = t_v;
h_vec = h_v;

% begin plotting
subplot(3, 1, 1);
ylabel("vertical displacement (m)");
xlabel("time (sec)");

if state == "air"
    plot(t_v, h_v(:,1), 'r-');
elseif state == "ground"
    plot(t_v, h_v(:,1), 'b-');
end
hold on;

% Various event stages
while t_vec(end) <= tfinal
    
    if isempty(h_e)
        break;
    end
    
    if state == "air"
        options = odeset('Events', @contactGroundEvent);
        [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
            [t_e tfinal], [h_e(1); h_e(2); h_e(3)], options);
        state = "ground";
    else
        options = odeset('Events', leaveGroundEventHandler);
        [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
            [t_e tfinal], [h_e(1); h_e(2); h_e(3)], options);
        state = "air";
    end

    if state == "air"
        plot(t_v, h_v(:,1), 'r-');
        %plot(t_v, h_v(:,3), 'g-', 'LineWidth', 5);
    elseif state == "ground"
        plot(t_v, h_v(:,1), 'b-');
    end

    t_vec = [t_vec; t_v(2:end, :)];
    h_vec = [h_vec; h_v(2:end, :)];
end

% PLOT HEIGHT AND RESTING STATE

plot(t_vec, h_vec(:, 3) - ((m * g) / k), 'r--');  % plot equilibrium condition
plot(t_vec, h_vec(:, 3), 'b--');
ylabel("vertical position (m)");
xlabel("time (sec)");


subplot(3, 1, 2);
plot(t_vec, h_vec(:,2), 'b--');
ylabel("vertical velocity (m/sec)");
xlabel("time (sec)");

subplot(3, 1, 3);
plot(t_vec, interp1(linspace(0, 0.5, numel(La_dot)-1), La_dot(1:end-1), t_vec), 'k--');
xlim([0 tfinal]);
ylabel("control (m/sec)");
xlabel("time (sec)");

%% TO GENERATE SIMULATION %%

if simulation
    pause(1);
    close all;

    FPS = 60;                                       % defining framerate
    t_anim = 0:1/FPS:t_vec(end);                    % defining framerate
    h_anim = interp1(t_vec, h_vec(:, 1), t_anim);   % finding points at these frames
    la_anim = interp1(t_vec, h_vec(:, 3), t_anim);
    
    if save_video
        v = VideoWriter(video_name, "MPEG-4");
        v.FrameRate = FPS;
        open(v);
    end

    figure
    for iter = 1:numel(t_anim)
        plot([0 0], [max(0, (h_anim(iter) - la_anim(iter))) h_anim(iter)], ...
            'k--', 'LineWidth', k * 0.02);
        hold on;
        axis equal;
        axis([-1 1 -0.25 2]);
        
        yline(0, 'k-');
        plot(0, h_anim(iter), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
        drawnow;
        
        if save_video
            frame = getframe(gcf);
            writeVideo(v, frame);
        end
        hold off;
    end

    if save_video
        close(v);
    end
    close all;
end

