% massLegActuatorSim
% Roger Chen
% 2026-06-15
% Adapted from massLegSim code
% Adds a variable actuator length to the model.

clc
clear
close all

%% Define variables
simulation = false;
save_video = false;
video_name = "massLegActuatorJumpSim.mp4";

[g, L, m, k, c, dynamicL, dynL] = constants();
tfinal = 5;                         % Time (seconds)

h0 = 1.5;                           % Starting height (m)
hdot0 = -1;                         % Starting velocity (m/s)

%% Define dynamics equations
if dynamicL
    ground_dynamics = @(t, y) [ y(2);
        (((dynL(t) - y(1)) * (k/m)) +...   % spring force - length changes over time
        (appliedForce(t) / m) -...                      % actuator force
        ((c*y(2)) / m) - ...                            % dampening force
        g)];                                            % gravity
    disp("Using dynamic L");
else
    ground_dynamics = @(t, y) [ y(2);
        (((L - y(1)) * (k/m)) +...   % spring force - length changes over time
        (appliedForce(t) / m) -...                      % actuator force
        ((c*y(2)) / m) - ...                            % dampening force
        g)];                                            % gravity
end

air_dynamics = @(t, y)  [ y(2) ; -g];

%% Run simulation with ODE45 %%

% determine initial state:
if L >= h0
    % STATE IS GROUND
    options = odeset('Events', @leaveGroundEvent);
    [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
        [0 tfinal], [h0; hdot0], options);
    state = "air";      % done with ground time
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

% begin plotting
subplot(2, 1, 1);
hold on;
ylabel("vertical displacement (m)");
xlabel("time (sec)");

if state == "air"
    plot(t_v, h_v(:,1), 'r-');
elseif state == "ground"
    plot(t_v, h_v(:,1), 'b-');
end

% Various event stages
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

    if state == "air"
        plot(t_v, h_v(:,1), 'r-');
    elseif state == "ground"
        plot(t_v, h_v(:,1), 'b-');
    end

    t_vec = [t_vec; t_v(2:end, :)];
    h_vec = [h_vec; h_v(2:end, :)];
end

% PLOT HEIGHT AND RESTING STATE
if dynamicL
    plot(t_vec, dynL(t_vec) - ((m * g) / k), 'r--');  % plot equilibrium condition
    plot(t_vec, dynL(t_vec), 'b--');
else
    yline(L - ((m * g) / k), 'r--');                  % plot equilibrium condition
    yline(L, 'b--');
end

subplot(2, 1, 2);
plot(t_vec, h_vec(:,2), 'b--');
ylabel("vertical velocity (m/sec)");
xlabel("time (sec)");

%% TO GENERATE SIMULATION %%

if simulation
    pause(1);
    close all;

    FPS = 30;                                       % defining framerate
    t_anim = 1:1/FPS:t_vec(end);                    % defining framerate
    h_anim = interp1(t_vec, h_vec(:, 1), t_anim);   % finding points at these frames
    
    if save_video
        v = VideoWriter(video_name, "MPEG-4");
        v.FrameRate = FPS;
        open(v);
    end

    figure
    for iter = 1:numel(t_anim)
        if dynamicL
            plot([0 0], [max(0, (h_anim(iter) - dynL(t_anim(iter)))) h_anim(iter)], ...
                'k--', 'LineWidth', k * 0.02);
        else
            plot([0 0], [max(0, (h_anim(iter) - L)) h_anim(iter)], ...
                'k--', 'LineWidth', k * 0.02);
        end
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

