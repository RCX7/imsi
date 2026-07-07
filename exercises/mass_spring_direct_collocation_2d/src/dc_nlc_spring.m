% dc_nlc_spring.m
% Roger Chen
% 2026-06-18
% Nonlinear constraints for direct collocation
% Includes guesses for trajectory as well as control

function [g, geq, jc, jceq] = dc_nlc_spring(w)
    [grav, Lp, m, k, c, z0, zdot0, x0, xdot0,...
        tfinal, N, n_state_vars, dt, xdot_target] = constants();

    % allocate defect constraints
    if isa(w, "sym")
        trajectory_constraints = sym(zeros((n_state_vars * (N-1)), 1));
    else
        trajectory_constraints = zeros((n_state_vars * (N-1)), 1);
    end

    % populate defect constraints
    for state_num=1:n_state_vars                               % for each state
        %if state % if reach a stance point, need to find contact

        for iter=1:N-1                                       % for each knot point in the state (except the last one)
            current = w((state_num)*N + iter);
            next_guessed = w((state_num)*N + iter + 1);      % the next state as a decision

            % different computations for f(x, u)
            switch state_num
                case 1 % state = z (flight 1)
                    fk = w(4*N + iter);
                    fkp1 = w(4*N + iter + 1);
                    Tstate = w(end-2);
                case 2 % state = z (stance)
                    fk = w(5*N + iter);
                    fkp1 = w(5*N + iter + 1);
                    Tstate = w(end-1);
                case 3 % state = z (flight 2)
                    fk = w(6*N + iter);
                    fkp1 = w(6*N + iter + 1);
                    Tstate = w(end);
                case 4 % state = zdot (flight1)
                    fk = -grav; fkp1 = -grav;
                    Tstate = w(end-2);
                case 5 % state = zdot (stance)
                    toe_pos = w(8*N) + w(end-3);

                    la = w(13*N + iter);
                    zdist = w(2*N + iter);
                    xdist = w(8*N + iter) - toe_pos;
                    dist = norm([zdist xdist]);                     % toe to mass distance
                    u = w(iter);
                    dzdist = current;
                    dxdist = w(11*N + iter);
                    ddist = (xdist*dxdist + zdist*dzdist) / dist;   % computed via chain rule
                    Force = (k * (la - dist)) + (c * (u - ddist));
                    
                    next_la = w(13*N + iter + 1);
                    next_zdist = w(2*N + iter + 1);
                    next_xdist = w(8*N + iter + 1) - toe_pos;
                    next_dist = norm([next_zdist next_xdist]);
                    next_u = w(iter + 1);
                    next_dzdist = next_guessed;
                    next_dxdist = w(11*N + iter + 1);
                    next_ddist = (next_xdist*next_dxdist + next_zdist*next_dzdist) / next_dist;
                    next_Force = (k * (next_la - next_dist)) + (c * (next_u - next_ddist));
                    
                    zForce = Force * (zdist / dist); % z position of toe is always at 0, so just pos (pos - 0)
                    next_zForce = next_Force * (next_zdist / next_dist);
                    fk = ((zForce / m) - grav);
                    fkp1 = ((next_zForce / m) - grav);
                    Tstate = w(end-1);
                case 6 % state = zdot (flight2)
                    fk = -grav; fkp1 = -grav;
                    Tstate = w(end);
                case 7 % state = x (flight1)
                    fk = w(10*N + iter);
                    fkp1 = w(10*N + iter + 1);
                    Tstate = w(end-2);
                case 8 % state = x (stance)
                    fk = w(11*N + iter);
                    fkp1 = w(11*N + iter + 1);
                    Tstate = w(end-1);
                case 9 % state = x (flight 2)
                    fk = w(12*N + iter);
                    fkp1 = w(12*N + iter + 1);
                    Tstate = w(end);
                case 10 % state = xdot(flight 1)
                    fk = 0; fkp1 = 0;
                    Tstate = w(end-2);
                case 11 % state = xdot (stance)
                    toe_pos = w(8*N) + w(end-3);

                    la = w(13*N + iter);
                    zdist = w(2*N + iter);
                    xdist = w(8*N + iter) - toe_pos;
                    dist = norm([zdist xdist]);                     % toe to mass distance
                    u = w(iter);
                    dzdist = w(5*N + iter);
                    dxdist = current;
                    ddist = (xdist*dxdist + zdist*dzdist) / dist;   % computed with chain rule
                    Force = (k * (la - dist)) + (c * (u - ddist));
                    
                    next_la = w(13*N + iter + 1);
                    next_zdist = w(2*N + iter + 1);
                    next_xdist = w(8*N + iter + 1) - toe_pos;
                    next_dist = norm([next_zdist next_xdist]);
                    next_u = w(iter + 1);
                    next_dzdist = w(5*N + iter + 1);
                    next_dxdist = next_guessed;
                    next_ddist = (next_xdist*next_dxdist + next_zdist*next_dzdist) /next_dist;
                    next_Force = (k * (next_la - next_dist)) + (c * (next_u - next_ddist));
                    
                    xForce = Force * (xdist / dist); % z position of toe is always at 0, so just pos (pos - 0)
                    next_xForce = next_Force * (next_xdist / next_dist);
                    fk = ((xForce / m));
                    fkp1 = ((next_xForce / m));
                    Tstate = w(end-1);
                case 12 % state = xdot(flight 2)
                    fk = 0; fkp1 = 0;
                    Tstate = w(end);
                case 13 % state = la (stance)
                    fk = w(iter);
                    fkp1 = w(iter + 1);
                    Tstate = w(end-1);
            end
            delta_t = (Tstate / (N - 1));
            next_predicted = current + 0.5 * (fk + fkp1) * delta_t;
            trajectory_constraints((state_num - 1) * (N-1)  + iter) = next_guessed - next_predicted;
        end
    end

    % starting constraints
    start_at_z0 = w(N+1) - z0;
    start_in_air = -w(N+1) + Lp; 
    start_at_zdot0 = w(4*N + 1) - zdot0;
    start_at_x0 = w(7*N + 1) - x0;
    start_at_xdot0 = w(10*N + 1) - xdot0;
    start_at_la0 = w(13*N + 1) - Lp;

    starting_constraints = [%start_at_z0;
                            %start_at_zdot0;
                            start_at_x0;
                            % start_at_xdot0;
                            start_at_la0];

    % phase transition constraints
    toe_pos = w(8*N) + w(end-3);
    las = w(13*N+1:14*N);
    zdists = w(2*N+1:3*N);
    xdists = w(8*N+1:9*N) - toe_pos;
    dists = sqrt(zdists.^2 + xdists.^2);      % toe to mass distances
    us = w(1:N);
    dzdists = w(5*N+1:6*N);
    dxdists = w(11*N+1:12*N);
    ddists = (xdists.*dxdists + zdists.*dzdists) ./ dists;   % using chain rule
    
    ground_contact_1 = dists(1) - Lp;         % end of first flight phase (first distance has to equal Lp)
    begin_stance = ground_contact_1;
    
    forces = (k .* (las - dists)) + (c .* (us - ddists));
    flim = 15;
    limited_forces = forces - flim;
    positive_forces  = -(forces);
    begin_flight = positive_forces(end);      % last force needs to be 0

    dists_less_than_la = dists - las;
    phase_transition_constraints = [begin_stance;
                                    begin_flight;];

    border_constraints = [w(2*N) - w(2*N + 1);   % zf1 -> zs
                          w(3*N) - w(3*N + 1);   % zs -> zf2
                          w(5*N) - w(5*N + 1);   % dzf1 -> dzs
                          w(6*N) - w(6*N + 1);   % dzs -> dzf2
                          w(8*N) - w(8*N + 1);   % xf1 -> xs
                          w(9*N) - w(9*N + 1);   % xs -> xf2
                          w(11*N) - w(11*N + 1); % dxf1 -> dxs
                          w(12*N) - w(12*N + 1); % dxs -> dxf2
                          ];
    border_constraints = [starting_constraints;
                          border_constraints;
                          phase_transition_constraints;
                          ];

    % target_constraints
    meet_target_xdot = ((w(10*N) - w(7*N+1)) / (sum(w(end-2:end)))) - xdot_target;
    end_at_starting_height = w(4*N) - w(N+1);
    end_at_starting_zvelocity = w(4*N +1) - w(7*N);
    end_at_starting_xvelocity = w(10*N+1) - w(13*N);

    target_constraints = [meet_target_xdot;
                          end_at_starting_zvelocity;
                          end_at_starting_height;
                          end_at_starting_xvelocity;
                          ];

    no_control = w(1:N);

    end_time_tolerance = 0.1;
    nonzero_end_time = -sum(w(end-2:end)) + end_time_tolerance;

    % combining them
    g = [start_in_air;
         % end_above_ground;
         positive_forces;
         %limited_forces;
         dists_less_than_la;
         nonzero_end_time;
         ];

    geq = [trajectory_constraints;
           border_constraints;
           target_constraints;
           %no_control;
           ];

    if ~(isa(w, "sym"))
        jc = JC(w).';
        jceq = JCeq(w).';
        jc(abs(jc) < 1e-12) = 0;
        jceq(abs(jceq) < 1e-12) = 0;
    else
        jc = 0;
        jceq = 0;
    end
end
