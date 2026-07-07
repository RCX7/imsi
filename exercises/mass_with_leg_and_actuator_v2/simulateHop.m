% simulateHop.m
% Roger Chen
% 2026-06-17
% Simulates a single hop
% Returns ending height and velocity to ensure that

function [peaks, las, hfinal] = simulateHop(w)
    % objective: see the heights that it bounces up to
    

    [g, la0, m, k, c, h0, hdot0, tfinal] = constants();
    ground_dynamics = @(t, y) groundDynamics(t, y, w);
    air_dynamics = @(t, y)  airDynamics(t, y, w);
    leaveGroundEventHandler = @(t, y) leaveGroundEvent(t, y, w);

    if la0 >= h0
        % STATE IS GROUND
        options = odeset('Events', leaveGroundEventHandler);
        [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
            [0 tfinal], [h0; hdot0; la0], options);
        state = "air";
    else
        % STATE IS AIR
        options = odeset('Events', @contactGroundEvent);
        [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
            [0 tfinal], [h0; hdot0; la0], options);
        state = "ground";    % done with air time
    end

    t_vec = t_v;
    h_vec = h_v;

    while t_vec(end) <= tfinal
        
        if isempty(h_e)
            break;
        end
        
        if state == "air"
            options = odeset('Events', @contactGroundEvent);
            [t_v, h_v, t_e, h_e, ie] = ode45(air_dynamics,...
                [t_e tfinal], [h_e(1); h_e(2); h_e(3)], options);
            state = "ground";
        else
            options = odeset('Events', leaveGroundEventHandler);
            [t_v, h_v, t_e, h_e, ie] = ode45(ground_dynamics,...
                [t_e tfinal], [h_e(1); h_e(2); h_e(3)], options);
            state = "air";
        end
        t_vec = [t_vec; t_v(2:end, :)];
        h_vec = [h_vec; h_v(2:end, :)];
    end

    crossings = strfind(sign(h_vec(:, 2))', [1 -1]);
    pos = h_vec(:, 1);
    peaks = pos(crossings);
    
    las = h_vec(:, 3);
    las = las(crossings);

    hfinal = h_vec(end, 1);
end