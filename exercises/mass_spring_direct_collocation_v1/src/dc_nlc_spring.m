% dc_nlc_spring.m
% Roger Chen
% 2026-06-18
% Nonlinear constraints for direct collocation
% Includes guesses for trajectory as well as control

function [g, geq, jc, jceq] = dc_nlc_spring(w)
    [grav, Lp, m, k, c, h0, hdot0, tfinal, N, n_state_vars, dt] = constants();
    tanh_smooth = 1000;                                      % controls smoothness of smooth reLU

    % allocate defect constraints
    if isa(w, "sym")
        defect_constraints = sym(zeros((n_state_vars * (N-1)), 1));
    else
        defect_constraints = zeros((n_state_vars * (N-1)), 1);
    end

    % populate defect constraints
    for state_num=1:n_state_vars                               % for each state
        for iter=1:N-1                                       % for each knot point in the state (except the last one)
            current = w((state_num)*N + iter);
            next_guessed = w((state_num)*N + iter + 1);      % the next state as a decision

            % different computations for f(x, u)
            switch state_num
                case 1 % state = h
                    fk = w(2*N + iter);
                    fkp1 = w(2*N + iter + 1);
                case 2 % state = hdot
                    pos = w(N + iter);
                    la = w(3*N + iter);
                    u = w(iter);

                    contact_factor = 0.5 * (1 + tanh(tanh_smooth * (Lp - pos)));   % like a smooth reLU
                    Force = contact_factor * ((k * (la - pos)) + (c * (u - current)));
                    fk = ((Force / m) - grav);
                    
                    next_pos = w(N + iter + 1);
                    next_la = w(3*N + iter + 1);
                    next_u = w(iter + 1);

                    Force = contact_factor * ((k * (next_la - next_pos)) + (c * (next_u - next_guessed)));
                    fkp1 = ((Force / m) - grav);
                case 3 % state = la
                    fk = w(iter);
                    fkp1 = w(iter + 1);
            end
            delta_t = (w(end) / (N - 1));
            next_predicted = current + 0.5 * (fk + fkp1) * delta_t;
            defect_constraints((state_num - 1) * (N-1)  + iter) = next_guessed - next_predicted;
        end
    end

    % other constraints
    end_above_ground = -w(2*N);
    end_at_start = w(2*N) - h0;
    end_at_time = w(end) - 0.5;
    end_at_zero_velocity = w(3*N);
    end_with_same_length = w(4*N) - Lp;

    % hdots = w(2*N+1:3*N);
    % times = linspace(0, w(end), N);
    % peak_idxs = strfind(sign(hdots).', [-1 1 -1]);
    % peak_times = times(peak_idxs);
    %first_peak = peak_times(1);
    %end_after_one_hop = w(end) - first_peak;

    start_at_h0 = w(N + 1) - h0;
    start_at_hdot0 = w(2*N + 1) - hdot0; 
    start_at_la0 = w(3*N + 1) - Lp;

    no_control = w(1:N);
    %no_control = w(N+1:2*N) - Lp;

    % combining them
    g = [end_above_ground];

    geq = [defect_constraints;
           start_at_h0;
           start_at_hdot0;
           start_at_la0;
           %no_control;
           end_at_start;
           end_at_time;
           end_at_zero_velocity;
           end_with_same_length;
           %end_after_one_hop;
           ];
    if ~(isa(w, "sym"))
        jc = JC.';
        jceq = JCeq(w).';
    else
        jc = 0;
        jceq = 0;
    end
end
