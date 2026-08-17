% plotTraj.m
% Roger Chen
% 2026-06-30
% To plot some trajectory

function [] = plotTraj(w, color, mode)

    [m, g, ~, ~, ~, ~, ~, ~, ~, t_sim, l_uN] = physConstants(mode);
    [N, ~] = simConstants();
    [control, states, addDecs] = decToMats(w);
    [xs, xdots, zs, zdots, las] = extractStates(states);

    %% POSITION / KINEMATICS
    subplot(2,2,1)
    hold on;
    plot(0, 0, "Marker","+", "Color","r", "LineWidth",3);
    plot(xs, zs, (color + "o-"));
    [xf, ~, zf, ~] = flightKinematics(xs(end), xdots(end),...
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
    plot(t_sim * linspace(0, addDecs(1), N), las * l_uN, (color + "o-"));
    xlim padded;
    ylim padded;
    % ylim([la0-laRange, la0+laRange]);

    title("Actuated Length");
    xlabel("Time (sec)");
    ylabel("Length (m)");
    hold off;

    %% FORCE PROFILE
    subplot(2, 2, 3) 
    forces = computeForces(states, control, mode);
    forces = forces / (m * g);
    total_impulse = trapz(addDecs(1) / (numel(forces) - 1), forces);
    hold on;
    plot(linspace(0, addDecs(1) * t_sim, N), forces, (color + "o-"));
    xlim padded;
    ylim padded;

    title("Force Profile (over time) | Total impulse: " + total_impulse);
    xlabel("Time (sec)");
    ylabel("Force (BW)");
    hold off;
    
    %% CONTROL
    subplot(2, 2, 4) 
    hold on;
    plot(t_sim * linspace(0, addDecs(1), N), control * (l_uN / t_sim), (color + "o-"));
    xlim padded;
    ylim padded;
    % ylim([-5, 5]);

    title("Control (Actuated Velocity)");
    xlabel("Time (sec)");
    ylabel("Velocity (m / s)");
    hold off;
end