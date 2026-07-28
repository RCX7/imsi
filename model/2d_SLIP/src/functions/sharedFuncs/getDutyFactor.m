% getDutyFactor.m
% Roger Chen
% 2026-07-21
% Computes the duty factor for a given w_star

function [df] = getDutyFactor(w)
    [control, states, addDec] = decToMats(w);
    
    t_stance = addDec(1);
    t_flight = addDec(2);
    t_stride = t_stance + t_flight;
    df = t_stance / (2 * t_stride);
end