% contactGroundEvent.m

function [value, isterminal, direction] = contactGroundEvent(t, y)
    L = 0.5;
    value = y(1) - L;
    isterminal = 1;
    direction = -1;
end