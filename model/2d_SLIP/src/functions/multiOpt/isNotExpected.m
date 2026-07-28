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

function flag = isNotExpected(cot)
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
    flag = (cot > 8 | cot <= 0);
end