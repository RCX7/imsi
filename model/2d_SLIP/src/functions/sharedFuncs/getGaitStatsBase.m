function [COT, stride_freq, stride_len, stride_duration, dutyfac, peak_force, impulse, fAng, vAng, cAng] =...
    getGaitStatsBase(w_star, gait, m, g, k, c, I, t_sim, l_uN, output_data)

    % Retrieves the characteristics of a gait described by w_star
    %   Returns values such as stride frequency, length, and duty factor

    arguments
        w_star
        gait
        m
        g
        k
        c
        I
        t_sim
        l_uN 
        output_data logical = false
    end
    
    [N, n_states] = simConstants();
    [control, states, addDecs] = decToMats(w_star);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    [xf, ~, ~, ~] = flightKinematics(xs(end), xdots(end),...
        zs(end), zdots(end), 0:0.01:addDecs(2));

    COT = costFunBase(w_star, m, g, I, gait);
    stride_duration = addDecs(1) * t_sim;
    stride_freq = 1 / (stride_duration);
    stride_len = (xf - xs(1)) * l_uN;
    dutyfac = getDutyFactor(w_star);
    if gait == 'h', dutyfac = dutyfac * 2; end
    
    forces = computeForcesBase(xs, zs, xdots, zdots, las, control, k, c);
    forces = forces / (m * g);
    peak_force = max(forces);
    impulse = trapz((addDecs(1) * t_sim) / N, forces);  % double check this?

    [fAng, vAng, cAng] = getCollAngles(w_star, k, c);


    if output_data
        un_speed = (stride_len / stride_duration);
        fprintf("Frequency: %.2f steps/s\n", stride_freq);
        fprintf("Speed: %.2f m/s (%.2f mph)\n", un_speed, un_speed * 2.23694)
        
        fprintf("COST: %.2f\n", COT);
        disp("Collision-based analysis angles (rad): ")
        fprintf("force: %f\n", fAng);
        fprintf("vel: %f\n", vAng);
        fprintf("coll: %f\n", cAng);
        fprintf("Stride Frequency: %.2f 1/sec\n", stride_freq);
        fprintf("Stride Length: %.2f m\n", stride_len);
        fprintf("Stride Duration: %.2f sec\n", stride_duration);
        fprintf("Duty Factor: %.2f\n", dutyfac);
        fprintf("Peak Force: %.2f BW\n", peak_force);
        fprintf("Impulse: %.2f BW*sec\n", impulse);
    end
end