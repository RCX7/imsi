% robustMinCOT.m
% Roger Chen
% 2026-07-07
% Runs minCOT until a reasonable solution is reached. Returns inf for
% everything if not.

function [COT, stanceCost, flightCost, tstance, tflight, next_guess] =...
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

    counter = 0;
    while isNotExpected(COT)
        if counter <= 3
            [COT, stanceCost, flightCost, tstance, tflight, next_guess] =...
                minCOT(mode, target_speed, target_freq, k, c, I, next_guess);
        elseif counter <= 5
            [COT, stanceCost, flightCost, tstance, tflight, next_guess] =...
                minCOT(mode, target_speed, target_freq, k, c, I, 0);
        else
            COT = inf; stanceCost = inf; flightCost = inf;
            tstance = inf; tflight = inf; next_guess = 0;
            break;
        end
        counter = counter + 1;
    end
end