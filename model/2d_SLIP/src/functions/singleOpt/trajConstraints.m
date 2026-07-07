% trajConstraints.m
% Roger Chen
% 2026-06-30
% Generating constraints for fmincon solver.

function [c, ceq] = trajConstraints(w, mode)

    % retrieve constants
    [m, g, k, c, la0, laRange, xdot_target, I] = physConstants(mode);
    [N, n_states] = simConstants();
    [control, states, addDecs] = decToMats(w);
    assert(all(size(addDecs) == [1, 2+N]));
    [xs, xdots, zs, zdots, las] = extractStates(states);
    forces = computeForces(states, control, mode);
    
    % compute defect constraints for trajectory kinematics
    dt = addDecs(1) / (N - 1);
    slopes = computeSlopes(states, control, mode);
    defect_constraints = states(:, 2:end) - (states(:, 1:end-1) + slopes * dt);
    defect_constraints = defect_constraints(:);

    % compute boundary constraints
    start_constraints = [sqrt(xs(1)^2 + zs(1)^2) - las(1);  % inital distance has to be length of leg
                         las(1) - la0;                      % initial length of leg has to be passive length
                         control(1)];
    
    [xf, xdotf, zf, zdotf] = flightKinematics(xs(end), xdots(end), zs(end), zdots(end), addDecs(2));
    match_target_speed = ((xf - xs(1)) / (addDecs(1) + addDecs(2))) - xdot_target;
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
    true_power = ((forces .* control) / (m*g*dist)) + 0;
    unsignedPowerConst = [-guessed_power + true_power;
                          -guessed_power - true_power;];

    c = [unsignedPowerConst;
        %-forces'
        ];

    ceq = [defect_constraints;
           border_constraints;
           %control'
           ];


end