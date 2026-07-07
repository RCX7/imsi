% contactGroundEvent.m

function [value, isterminal, direction] = contactGroundEvent(t, y)
    
    value = y(1) - y(3);        % contacts the ground when the height = actuated length
                                % actuated length is constrained to be Lp
                                % but that can be done later in the sim.
    isterminal = 1;
    direction = -1;
end