% matToDec.m
% Roger Chen
% 2026-06-30
% converts a matrix and a control vector into a single decision vector.
% Control is at the beginning of the decision vector, followed by the
% states. Additional decisions are included as an optional parameter.

function [w] = matToDec(control, states, addDec)
    arguments (Input)
        control (1, :)
        states  (:, :)
        addDec  (1, :) = []    % Optional (defaults to empty row)
    end
    arguments (Output)
        w (:, 1)               % returns a column vector
    end
    
    tstates = states.';
    w = [control(:); tstates(:); addDec(:)];

end