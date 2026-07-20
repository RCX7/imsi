% getCollAngles.m
% Roger Chen
% 2026-07-16
% Reports collision angles for a given decision vector

function [vel_angle, force_angle, coll_angle] = getCollAngles(w)
    [control, states, addDec] = decToMats(w);
    [xs, xdots, zs, zdots, las] = extractStates(states);
    
    V_mag = sqrt(xdots(1)^2 + zdots(1)^2);
    la0 = sqrt(xs(1)^2 + zs(1)^2);

    vel_angle = acos(xdots(1) / V_mag);
    force_angle = acos(zs(1) / la0);
    coll_angle = asin(abs(cos(vel_angle + force_angle + (pi / 2))));
end