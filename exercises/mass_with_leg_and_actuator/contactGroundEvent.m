% contactGroundEvent.m

function [value, isterminal, direction] = contactGroundEvent(t, y)
    [g, L, m, k, c, dynamicL, dynL] = constants();

    %value = -(((L - y(1)) * k) + (appliedForce(t)) - (c * y(2)));
    if dynamicL
        value = y(1) - dynL(t);
    else
        value = y(1) - L;
    end
    isterminal = 1;
    direction = -1;
end