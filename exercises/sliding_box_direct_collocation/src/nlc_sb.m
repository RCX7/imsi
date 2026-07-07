% nlc_sb.m
% Roger Chen
% 2026-06-19
% Creates the constraints (defect and regular) for sliding box problem

function [g, geq, Jc, Jceq] = nlc_sb(w)

    [grav, m, s_final, N, num_state_vars] = constants();
    Tf = w(end);
    dt = Tf / (N - 1);

    % defect constraints first
    if isa(w, "sym")
        defect_constraints = sym(zeros(num_state_vars * (N-1), 1));
    else
        defect_constraints = zeros(num_state_vars * (N-1), 1);
    end
    for state_num=1:num_state_vars
        for iter=1:N-1
            % get guess
            guess = w(state_num*N + iter + 1);
            current = w(state_num*N + iter);
            switch state_num
                case 1 % position
                    fk = w(2*N + iter);
                    fkp1 = w(2*N + iter + 1);
                case 2 % velocity
                    fk = w(iter) / m;
                    fkp1 = w(iter + 1) / m;
            end
            predicted = current + 0.5 * (fk + fkp1) * dt;
            % set defect constraints
            %disp(guess - predicted);
            defect_constraints((N - 1)*(state_num - 1) + iter) = guess - predicted;
        end
    end
    
    %disp(defect_constraints);

    % non defect constraints
    positive_time = -w(end);
    start_at_rest = w(2*N+1);
    start_at_0 = w(N+1);
    end_at_rest = w(3*N);
    end_at_target = w(2*N) - s_final;

    % add them together
    g = [positive_time];
    %disp(defect_constraints);
    geq = [start_at_rest;
           start_at_0;
           end_at_rest;
           end_at_target;
           defect_constraints];
    if ~(isa(w, "sym"))
        Jc = JC.';
        Jceq = JCeq(w).';
    else
        Jc = 0;
        Jceq = 0;
    end
    
    % disp(class(g));
    % disp(class(geq));
    % disp(class(Jc));
    % disp(class(Jceq));
end