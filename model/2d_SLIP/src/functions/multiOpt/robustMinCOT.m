% robustMinCOT.m
% Roger Chen
% 2026-07-07
% Runs minCOT until a reasonable solution is reached. Returns inf for
% everything if not.

function [COT, stanceCost, flightCost, tstance, tflight, next_guess, cost_scale_factor] =...
    robustMinCOT(mode, target_speed, target_freq, k, c, I, w0)
    
    arguments
        mode char
        target_speed
        target_freq
        k
        c
        I
        w0 = 0;
    end
    next_guess = w0;
    COT = 0;
    tstance = 0; tflight = 0;

    g_uN = 9.81;
    l_uN = 1.15;
    t_sim = sqrt(l_uN / g_uN);
    counter = 0;
    while isNotExpected(COT, (t_sim * (tstance + tflight)))
        if counter <= 3  % ADJUST AS NEEDED
            [COT, stanceCost, flightCost, tstance, tflight, next_guess, cost_scale_factor] =...
                minCOT(mode, target_speed, target_freq, k, c, I, next_guess);
        elseif counter <= 5
            [COT, stanceCost, flightCost, tstance, tflight, next_guess, cost_scale_factor] =...
                minCOT(mode, target_speed, target_freq, k, c, I, 0);
        else
            COT = inf; stanceCost = inf; flightCost = inf;
            tstance = inf; tflight = inf; next_guess = 0; cost_scale_factor = 0;
            break;
        end
        counter = counter + 1;
    end
end