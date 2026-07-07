% constants.m
% a way to access constants globally.

function [g, L, m, k, c, dynamicL, dynL] = constants()
    g = 9.81;           % Gravity (m/s^2)
    L = 0.8;            % Leg length (meters)
    m = 0.2;              % Mass (kg)
    k = 50;            % Spring constant (N/m)
    c = 0.5;            % Dampening constant (Ns/m)
    dynamicL = true;
    if dynamicL
        %dynL = @(t) L + 0.2 * sin(3 * t);   % arbitrary function - will be decision variable.
        dynL = @dynLHelper; 
    else
        dynL = @(t) L * ones(size(t)); 
    end

    % --- VECTOR-SAFE NESTED FUNCTION ---
    function newL = dynLHelper(t)
        newL = L * ones(size(t));  
        mask = (0.3 < t & t < 0.4) | (1.45 < t & t < 1.6);
        newL(mask) = 1.2 * L;
    end
end

