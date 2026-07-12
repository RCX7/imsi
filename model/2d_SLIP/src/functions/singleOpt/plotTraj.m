% plotTraj.m
% Roger Chen
% 2026-06-30
% To plot some trajectory

function [] = plotTraj(w, color, mode)

    [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode);
    [N, n_states] = simConstants();
    [control, states, addDecs] = decToMats(w);
    [xs, xdots, zs, zdots, las] = extractStates(states);

    %% POSITION / KINEMATICS
    subplot(2,2,1)
    hold on;
    plot(0, 0, "Marker","+", "Color","r", "LineWidth",3);
    plot(xs, zs, (color + "o-"));
    [xf, xdotf, zf, zdotf] = flightKinematics(xs(end), xdots(end),...
        zs(end), zdots(end), 0:0.01:addDecs(2));
    plot(xf, zf, "b");
    xlim padded;
    ylim padded;
    % ylim([0, 1.5]);

    title("Stance kinematics");
    xlabel("X position");
    ylabel("Z position");
    hold off;
    
    %% ACTUATED LENGTH
    subplot(2, 2, 2)
    hold on;
    plot(linspace(0, addDecs(1), N), las, (color + "o-"));
    xlim padded;
    ylim([la0-laRange, la0+laRange]);

    title("Actuated Length");
    xlabel("Time (sec)");
    ylabel("Length (m)");
    hold off;

    %% FORCE PROFILE
    subplot(2, 2, 3) 
    forces = computeForces(states, control, mode);
    forces = forces / (m * g);
    hold on;
    plot(linspace(0, addDecs(1), N), forces, (color + "o-"));
    xlim padded;
    ylim padded;

    title("Force Profile (over time)");
    xlabel("Time (sec)");
    ylabel("Force (BW)");
    hold off;
    
    %% CONTROL
    subplot(2, 2, 4) 
    hold on;
    plot(linspace(0, addDecs(1), N), control, (color + "o-"));
    xlim padded;
    ylim([-5, 5]);

    title("Control (Actuated Velocity)");
    xlabel("Time (sec)");
    ylabel("Velocity (m/s)");
    hold off;
end