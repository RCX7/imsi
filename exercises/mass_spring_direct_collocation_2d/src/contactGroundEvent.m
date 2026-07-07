% contactGroundEvent.m

function [value, isterminal, direction] = contactGroundEvent(t, y, toe_pos)

    xdist = toe_pos;
    zdist = y(1);
    dist = sqrt(xdist^2 + zdist^2);
    value = dist - y(5);        % contacts the ground when the height = actuated length
                                % actuated length is constrained to be Lp
                                % but that can be done later in the sim.
    isterminal = 1;
    direction = -1;
end