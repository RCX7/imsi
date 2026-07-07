% leaveGroundEvent
function [value, isterminal, direction] = leaveGroundEvent(t, y, w, x0)
    [g, la0, m, k, c, h0, hdot0, tfinal] = constants();

    % interpolate w
    Tf = w(end-1);
    if t < Tf
        N = numel(w) - 2;
        t_itrp = linspace(0, Tf, N);
        u = interp1(t_itrp, w(1:end-2), t);           % La_dot (control signal) at point t
    else
        u = 0;
    end
    
    toe_pos = x0 + w(end);
    xdist = y(3) - toe_pos;
    zdist = y(1);
    dist = sqrt(xdist^2 + zdist^2);
    dxdist = y(4); dzdist = y(2);
    ddist = (xdist*dxdist + zdist*dzdist) / dist;
    F = k * (y(5) - dist) + c * (u - ddist);       % Based on this definition y(3) is the length of the spring
    value = F;                                     % force equation
    isterminal = 1;
    direction = -1;                                % when going from high to low (negative direction)
end