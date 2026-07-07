% leaveGroundEvent
function [value, isterminal, direction] = leaveGroundEvent(t, y, w)
    [g, la0, m, k, c, h0, hdot0, tfinal] = constants();

    % interpolate w
    Tf = w(end);
    if Tf > t
        N = numel(w) - 1;
        t_itrp = linspace(0, Tf, N);
        u = interp1(t_itrp, w(1:end-1), t);           % La_dot (control signal) at point t
    else
        u = 0;
    end
    
    value = k * (y(3) - y(1)) + c * (u - y(2));   % force equation
    isterminal = 1;
    direction = -1;                               % when going from high to low (negative direction)
end