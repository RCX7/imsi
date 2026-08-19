% physCconstants.m
% Roger Chen
% 2026-06-30
% Modularizing commonly used physicaal constants. Used in determining the
% kinematics of the simulation

function [m, g, k, c, la0, laRange, target_speed, target_freq, I, t_sim, l_uN]...
            = physConstants(mode)
    assert(mode == 'r' | mode == 'h')
    
    %% UNNORMALIZED (PHYSICAL) BASE SCALES
    g_uN = 9.81;        % m/s^2
    % m_uN = 80;          % kg (example body mass)
    l_uN = 1.15;        % m (unnormalized leg length)
    % l_uN = 0.8;
    
    % Derived unit scales
    t_sim = sqrt(l_uN / g_uN);              % Time scale (seconds)
    v_scale = (l_uN / t_sim);               % Velocity scale (m/s)
    % k_scale = (m_uN * g_uN) / l_uN;       % Stiffness scale (N/m)
    % c_scale = m_uN / t_sim;               % Damping scale (N*s/m)
    % I_scale = m_uN * (l_uN^2);            % Inertia scale (kg*m^2)

    %% UNNORMALIZED TARGET INPUTS
    target_speed_uN = 1;                % m/s
    target_freq_uN  = 0;                % Hz
    if mode == 'r', target_freq_uN = target_freq_uN * 2; end
    
    %% DIMENSIONLESS CONSTANTS (SIMULATION UNITS)
    m = 1;
    g = 1;
    la0 = 1;
    laRange = 10;
    
    target_speed = target_speed_uN / v_scale; 
    target_freq  = target_freq_uN * t_sim;    
    % I = 0.0463;             % Dimensionless inertia I*
    % I = 0.019;              % recomputed leg inertia cost (considering leg bend)
    I = 0.019;
    % I = 0.011;

    if mode == 'r'
        % unconstrained
        % k = 27.5; c = 0.1;
        % constrained
        k = 22.5; c = 0.1;
        k = 25; c = 0.5;   % refit 20260818 with constant I
        % k = 25; c = 0.225; I = 0.011;
    elseif mode == 'h'
        % unconstrained
        % k = 20; c = 0.15;
        % k = 25; c= 0.3;
        k = 20; c = 0.45;   % refit 20260818 with constant I
        I = I * 2;  % arbitrary estimation of added leg cost to hopping - knee tuck??

        % k = 20; c = 0.05; I = 0.012;
    end
end
