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
                case 1 % state = h (flight 1)
                    fk = w(4*N + iter);
                    fkp1 = w(4*N + iter + 1);
                    Tstate = w(end-2);
                case 2 % state = h (stance)
                    fk = w(5*N + iter);
                    fkp1 = w(5*N + iter + 1);
                    Tstate = w(end-1);
                case 3 % state = h (flight 2)
                    fk = w(6*N + iter);
                    fkp1 = w(6*N + iter + 1);
                    Tstate = w(end);
                case 4 % state = hdot (flight1)
                    fk = -grav;
                    fkp1 = -grav;
                    Tstate = w(end-2);
                case 5 % state = hdot (stance)
                    pos = w(2*N + iter);
                    la = w(7*N + iter);
                    u = w(iter);
                    vel = current;
                    Force = (k * (la - pos)) + (c * (u - vel));
                    
                    next_pos = w(2*N + iter + 1);
                    next_la = w(7*N + iter + 1);
                    next_u = w(iter + 1);
                    next_vel = next_guessed;
                    next_Force = (k * (next_la - next_pos)) + (c * (next_u - next_vel));

                    fk = ((Force / m) - grav);
                    fkp1 = ((next_Force / m) - grav);
                    Tstate = w(end-1);
                case 6 % state = hdot (flight2)
                    fk = -grav;
                    fkp1 = -grav;
                    Tstate = w(end);
                case 7 % state = la (stance)
                    fk = w(iter);
                    fkp1 = w(iter + 1);
                    Tstate = w(end-1);
            end
            delta_t = (Tstate / (N - 1));
            next_predicted = current + 0.5 * (fk + fkp1) * delta_t;
            defect_constraints((state_num - 1) * (N-1)  + iter) = next_guessed - next_predicted;
        end
    end

    % border constraints
    start_at_h0 = w(N + 1) - h0;
    start_at_hdot0 = w(4*N + 1) - hdot0; 
    start_at_la0 = w(7*N + 1) - Lp;
    ground_contact_1 = w(2*N) - Lp;                 % end of first flight phase
    begin_stance = ground_contact_1;
    
    pos = w(2*N+1:3*N);
    la = w(7*N+1:8*N);
    u = w(1:N);
    vel = w(5*N+1:6*N);
    positive_forces  = -((k .* (la - pos)) + (c .* (u - vel)));
    begin_flight = positive_forces(end);
    
    % if isa(w, "sym")
    %     border_constraints = sym(zeros(4, 1));
    % else
    %     border_constraints = zeros(4, 1);
    % end
    % % ensure that positions and velocities line up like in multi shooting
    % bc_counter = 1;
    % for state_num=[2 3 5 6]
    %     current = w((state_num)*N);
    %     next_guessed = w((state_num)*N + 1);
    % 
    %     % different computations for f(x, u)
    %     switch state_num
    %         case 2 % state = h (stance)
    %             fk = w(5*N);
    %             fkp1 = w(5*N + 1);
    %             Tstate = w(end-1);
    %         case 3 % state = h (flight 2)
    %             fk = w(6*N);
    %             fkp1 = w(6*N + 1);
    %             Tstate = w(end);
    %         case 5 % state = hdot (stance)
    %             pos = w(2*N+1);
    %             la = w(7*N+1);
    %             u = w(1);
    %             vel = current;
    %             Force = (k * (la - pos)) + (c * (u - vel));
    % 
    %             fk = ((Force / m) - grav);
    %             fkp1 = -grav;
    %             Tstate = w(end-1);
    %         case 6 % state = hdot (flight2)
    %             fk = -grav;
    %             fkp1 = -grav;
    %             Tstate = w(end);
    %     end
    %     delta_t = (Tstate / (N - 1));
    %     next_predicted = current + 0.5 * (fk + fkp1) * delta_t;
    %     border_constraints(bc_counter) = next_guessed - next_predicted;
    %     bc_counter = bc_counter + 1;
    % end

    border_constraints = [
                          w(2*N) - w(2*N + 1); % hf1 -> hs
                          w(3*N) - w(3*N + 1); % hs -> hf2
                          w(5*N) - w(5*N + 1); % dhf1 -> dhs
                          w(6*N) - w(6*N + 1); % dhs -> dhf2
                          ];
    border_constraints = [border_constraints;
                          start_at_h0;
                          start_at_hdot0;
                          start_at_la0;
                          begin_stance;
                          begin_flight;
                          ];

    % target_constraints
    end_above_ground = -w(2*N);
    end_at_time = (w(end-2) + w(end-1) + w(end)) - 0.5;
    end_at_zero_velocity = w(7*N);
    end_at_start = w(4*N) - h0;
    end_at_starting_length = w(8*N) - Lp;

    no_control = w(1:N);

    % combining them
    g = [end_above_ground;
         positive_forces];

    geq = [defect_constraints;
           border_constraints;
           end_at_zero_velocity;
           end_at_start;
           %end_at_starting_length;
           %no_control;
           ];

    if ~(isa(w, "sym"))
        jc = JC.';
        jceq = JCeq(w).';
    else
        jc = 0;
        jceq = 0;
    end
end
