% leaveGroundEvent
function [value, isterminal, direction] = leaveGroundEvent(t, y)
    [g, L, m, k, c, dynamicL, dynL] = constants();
    
    if dynamicL
        value = (((dynL(t) - y(1)) * k) + (appliedForce(t)) - (c * y(2)));
    else
        value = (((L - y(1)) * k) + (appliedForce(t)) - (c * y(2)));
    end
    isterminal = 1;
    direction = -1;          % when going from low to high (positive direction)
end