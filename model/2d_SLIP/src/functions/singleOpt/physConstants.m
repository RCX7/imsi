% physCconstants.m
% Roger Chen
% 2026-06-30
% Modularizing commonly used physical constants. Used in determining the
% kinematics of the simulation

function [m, g, k, c, la0, laRange, target_speed, target_freq, I, t_sim, l_uN]...
            = physConstants(mode)
    assert(mode == 'r' | mode == 'h')
    
    %% UNNORMALIZED (PHYSICAL) BASE SCALES
    g_uN = 9.81;        % m/s^2
    % m_uN = 80;          % kg (example body mass)
    l_uN = 1.15;        % m (unnormalized leg length)
    % l_uN = 0.85;          % unnormalized leg length for S06
    
    % Derived unit scales
    t_sim = sqrt(l_uN / g_uN);              % Time scale (seconds)
    v_scale = (l_uN / t_sim);               % Velocity scale (m/s)
    % k_scale = (m_uN * g_uN) / l_uN;       % Stiffness scale (N/m)
    % c_scale = m_uN / t_sim;               % Damping scale (N*s/m)
    % I_scale = m_uN * (l_uN^2);            % Inertia scale (kg*m^2)

    %% UNNORMALIZED TARGET INPUTS
    target_speed_uN = 2;                % m/s
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

    if mode == 'r'
        % hop run constrained two parameter fits
        % S05 2ms
        k = 20;
        c = 0.1;
        I = 0.01;

        % S05 1ms - not precise
        % k = 15;
        % c = 0.25;
        % I = 0.01;

        % S06 1ms - probably not precise but don't have 2ms data
        % k = 20;
        % c = 0.025;
        % I = 0.01;

        % two parameter fits
        % k = 15; c = 0.4;   % FOR S05, I = 0.06
        % k = 17.50; c = 0.1;  % FOR S06, I = 0.06

        % three parameter fits
        % k = 25; c = 0.05; I = 0.06;
        % k = 25; c = 0.2250; I = 0.0110;   % S05
        % k = 12.5; c = 0.05; I = 0.009;     % S06 - seems to work with cost sweeps... but gait predictions are slightly off
        % k = 20; c = 0.05; I = 0.012;     % S06 - experimenting with k
        % k = 15; c = 0.025; I = 0.01;   % S06 newest cost function
    elseif mode == 'h'
        % return to doubling parameters
        % S05 2ms
        % k = 40;
        % c = 0.2;
        % I = 0.02;
        
        % S05 2ms full stride fit
        k = 47.5;
        c = 0.15;
        I = 0.06;

        % two parameter fits
        %k = 20; c = 0.15;
        % k = 12.5; c = 0.35;
        % k = 15; c = 0.38;    % FOR S05, I = 2.5 * 0.06 = 0.15
        % k = 35; c = 0.38;
        % % k = 25; c= 0.15;     % FOR S06, I = 2.5 * 0.06 = 0.15
        % I = 2.15 * I;
        
        % three parameter fits
        % k = 20; c = 0.05; I = 0.012;
        % k = 20; c = 0.225; I = 0.022;   % S05
        % k = 20; c = 0.025; I = 0.03;     % S06
    end
end
