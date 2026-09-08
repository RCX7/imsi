% isNotExpected.m
% Roger Chen
% 2026-07-07
% Returns a boolean value for whether a value is expected based on
% previously computed values. Used to automate recomputing
% Inputs:
%   - cot: current COT
%   - prevCOTS: previous COTs
% Outputs:
%   - flag: true if not expected

function flag = isNotExpected(cot, ttotal)

    cot_cond = (cot > 20 | cot <= 0.005);
    time_cond = (ttotal > 5 | ttotal < 0.005);
    flag = cot_cond | time_cond;
    
    if (cot_cond && cot ~= 0)
        fprintf("COT condition failed: %.3f\n", cot);
    elseif (time_cond && ttotal ~= 0)
        fprintf("Time condition failed: %.3f\n", ttotal);
    end
end