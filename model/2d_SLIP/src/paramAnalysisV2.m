% add necessary paths
addpath('functions/multiOpt');
addpath('functions/sharedFuncs');
addpath('warmStartTemplates');
addpath('data');


% some constants
m = 1;
c = 2.8;
I = 2e-4;

speeds = 1.5:0.25:3.5;
ks = 500:100:900;

% BASED ON PARALLELIZE CODE ABOVE

num_ks = length(ks);
num_speeds = length(speeds);


% BASED ON PARALLELIZE CODE ABOVE

num_ks = length(ks);
num_speeds = length(speeds);

% 1. Preallocate matrices for parallel efficiency and sliced variable compliance
running_COTs = zeros(num_ks, num_speeds);
hopping_COTs = zeros(num_ks, num_speeds);

parfor k_idx = 1:num_ks
    k = ks(k_idx);
    
    % 2. Reset next_guess for each worker to break the inter-loop dependency
    next_guess = 0; 
    
    % Initialize local row accumulators for the inner sequential loop
    runCOT_tmpRow = [];
    hopCOT_tmpRow = [];
    
    for s_idx = 1:num_speeds
        speed = speeds(s_idx);
        
        % --- Running COT ---
        runCOT = 0;
        counter = 0;
        while isNotExpected(runCOT, runCOT_tmpRow)
            if counter <= 3
                [runCOT, next_guess] = minCOT('r', speed, k, c, I, next_guess);
            elseif counter <= 5
                [runCOT, next_guess] = minCOT('r', speed, k, c, I, 0);
            else
                runCOT = inf; % mark for manual run
                break;
            end
            counter = counter + 1;
        end
        runCOT_tmpRow = [runCOT_tmpRow, runCOT];
        
        % --- Hopping COT ---
        hopCOT = 0;
        counter = 0;
        while isNotExpected(hopCOT, hopCOT_tmpRow)
            if counter <= 3
                [hopCOT, next_guess] = minCOT('h', speed, k, c, I, next_guess);
            elseif counter <= 5
                [hopCOT, next_guess] = minCOT('h', speed, k, c, I, 0);
            else
                hopCOT = inf; % mark for manual run
                break;
            end
            counter = counter + 1;
        end
        hopCOT_tmpRow = [hopCOT_tmpRow, hopCOT];
    end
    
    % 3. Assign rows to sliced variables so MATLAB can stitch them back together
    running_COTs(k_idx, :) = runCOT_tmpRow;
    hopping_COTs(k_idx, :) = hopCOT_tmpRow;
end