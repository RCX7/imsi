% leaveGroundEvent
function [value, isterminal, direction] = leaveGroundEvent(t, y)
    L = 0.5;            % Leg length (meters)
    value = y(1) - L;       % in this case no force is applied
    isterminal = 1;
    direction = 1;          % when going from low to high (positive direction)
end