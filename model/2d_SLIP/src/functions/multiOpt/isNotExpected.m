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
    % COMPLEX CALCULATION
    % prevCOTs = prevCOTs(prevCOTs ~= 0 & ~isinf(prevCOTs));
    % %disp(prevCOTs);
    % 
    % if cot <= 0
    %     flag = true;
    % elseif isempty(prevCOTs)
    %     flag = cot < 2; % take the first nonzero estimate less than 2
    % elseif isscalar(prevCOTs) % one element
    %     flag = (abs(cot - prevCOTs) > 0.1);  % 0.1 determined manually
    % else
    %     avg_diff = mean(diff(prevCOTs));
    %     flag = (abs(cot - prevCOTs(end)) > (2 * avg_diff));
    % end

    % SIMPLE CALCULATION - WORKS ALMOST AS WELL?
    cot_cond = (cot > 17.5 | cot <= 0.05);
    time_cond = (ttotal > 1.5 | ttotal < 0.1);
    flag = cot_cond | time_cond;
    
    if (cot_cond && cot ~= 0)
        fprintf("COT condition failed: %.3f\n", cot);
    elseif (time_cond && ttotal ~= 0)
        fprintf("Time condition failed: %.3f\n", ttotal);
    end
end