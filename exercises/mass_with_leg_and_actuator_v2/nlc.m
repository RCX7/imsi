function [g, geq] = nlc(w)
    % simulate hop to get constraint values
    % difference has to be within some tolerance range
    [g, la0, m, k, c, h0, hdot0, tfinal] = constants();

    % 1st constraint - has to hop within some height
    % desiredHeight = 1;                      % try to always hop to one meter
    % tolerance = 0.003;
    % peaks = simulateHop(w);
    % error = sum((peaks - desiredHeight).^2);    % simple mse
    % 
    % g = error - tolerance;
    
    % 
    [peaks, las, hfinal] = simulateHop(w);
    desiredPeak = 1;                      % try to always hop to one meter
    htol = 0.01;
    ltol = 0.01;
    
    g = [
         (-trapz(w) - la0),...
         (trapz(w) - 0.3),...
         (sum(diff(las).^2) - ltol),...
         (sum((peaks - desiredPeak).^2) - htol),... %(sum(diff(peaks).^2) - htol),...
         (-hfinal)
        ];
    geq = 0;

end