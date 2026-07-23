% getCollAngles.m
% Roger Chen
% 2026-07-16
% Reports collision angles for a given decision vector

function [fAng, vAng, cAng] = getCollAngles(w, k, c)
    [control, states, addDec] = decToMats(w);
    [xs, xdots, zs, zdots, las] = extractStates(states);

    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);
    dists = sqrt(xs.^2 + zs.^2);
    v_totals = sqrt(xdots.^2 + zdots.^2);

    theta_i = acos(zs ./ dists);
    lambda_i = acos(xdots ./ v_totals);
    phi_i = theta_i + lambda_i;

    fAng = sum(forces .* theta_i) / sum(forces);
    vAng = sum(v_totals .* lambda_i) / sum(v_totals);
    cAng = sum(forces .* v_totals .* phi_i) / sum(forces .* v_totals);
    

    % peak force analysis
    pk_frc_idx = find(forces == max(forces));
    disp("=== Peak Force angles: ===");
    fprintf("Force: %.5f\n", theta_i(pk_frc_idx));
    fprintf("Velocity: %.5f\n", lambda_i(pk_frc_idx));
    fprintf("Collision: %.5f\n", phi_i(pk_frc_idx));
    disp("==========================");
end