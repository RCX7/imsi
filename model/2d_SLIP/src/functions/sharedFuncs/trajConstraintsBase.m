% trajConstraintsBase.m
% Roger Chen
% 2026-07-28
% A base function to modularize the logic for the trajectory constraints.
% Derived versions exist for singleOpt and multiOpt

function [const, const_eq] = trajConstraintsBase(w, m, g, la0, target_speed, target_freq, k, c)

    % retrieve constants
    [N, ~] = simConstants();
    [control, states, addDecs] = decToMats(w);
    assert(all(size(addDecs) == [1, 2+N]));
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);
    total_time = addDecs(1) + addDecs(2);
    
    % compute defect constraints for trajectory kinematics
    dt = addDecs(1) / (N - 1);
    slopes = computeSlopesBase(xs, xdots, zs, zdots, las, control, m, g, k, c);
    defect_constraints = states(:, 2:end) - (states(:, 1:end-1) + slopes * dt);
    defect_constraints = defect_constraints(:);

    % compute boundary constraints
    start_constraints = [sqrt(xs(1)^2 + zs(1)^2) - las(1);  % inital distance has to be length of leg
                         las(1) - la0;                      % initial length of leg has to be passive length
                         control(1)];
    
    [xf, xdotf, zf, zdotf] = flightKinematics(xs(end), xdots(end), zs(end), zdots(end), addDecs(2));
    match_target_speed = ((xf - xs(1)) / (total_time)) - target_speed;
    end_constraints = [(xdots(1) - xdotf);
                       (zs(1) - zf);
                       (zdots(1) - zdotf);
                       match_target_speed
                       ];
    stance_to_flight_constraints = [forces(end)];
    border_constraints = [start_constraints;
                          stance_to_flight_constraints;
                          end_constraints];

    % cool trick for abs value
    guessed_power = addDecs(3:2+N);
    dist = xf - xs(1);
    true_power = ((forces .* control) ./ (m*g*dist));
    unsignedPowerConst = [-guessed_power + true_power;
                          -guessed_power - true_power;];
    
    frequency_constraints = [];
    if target_freq ~= 0
        frequency_constraints = [total_time - (1 / target_freq);]; % normalized
    end

    const = [
        unsignedPowerConst;
        % -forces;
        ];

    const_eq = [
           defect_constraints;
           border_constraints;
           frequency_constraints;
           %control'
           ];

end