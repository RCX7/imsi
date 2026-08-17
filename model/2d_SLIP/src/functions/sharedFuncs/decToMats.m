% decToMats.m
% Roger Chen
% 2026-06-30
% A function to convert a decision vector to a matrix of states and a
% control vector. Useful for going between the two when computing
% constraints, for example.

function [control, states, addDec] = decToMats(w, N)
    arguments (Input)
        w (:, 1)
        N (1,:) = 0;
    end
    arguments (Output)
        control (1, :)
        states  (:, :)
        addDec  (1, :)
    end
    
    [const_N, n_states] = simConstants();
    if N == 0, N = const_N; end
    control = w(1:N);
    states = w(N+1:(n_states + 1)*N);
    states = reshape(states,N,n_states);
    states = states.';
    addDec = w((n_states + 1)*N + 1:end);
end