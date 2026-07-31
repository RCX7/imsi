% avgForceCurves.m
% Roger Chen
% 2027-07-30
% Given force data, finds horizontal, vertical, and total force components
% for that data. Pulls from exp_data folder.

clc; clearvars; close all;

save_result = true;
DATAPATH = 'C:\Users\roger\imsi\exp_data\S01Data\ForceData';
disp("Data source: " + DATAPATH);
addpath(DATAPATH);

%% settings
gait='r';
trials = {'hop_1ms_4min'; 'run_1ms_4min'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
m = 92.65;      % body mass
g = 9.81;       % gravity
fs = 2000;      % sampling frequency

%% import and extract the data
force_data = importdata([curr_trial '.mot']);
bounds_mask = 1:18900;     % needs to be manually tuned.

t = force_data.data(:,1);
t = t(bounds_mask);
t_total = t(end) - t(1);

F_left = force_data.data(:, [2, 4, 3]);
F_right = force_data.data(:, [11, 13, 12]);
% F_left = F_left(bounds_mask); F_right = F_right(bounds_mask);
F_net = F_right;
% F_net = F_right + F_left;


%% find stride times
Fz_net = F_net(:, 3);         % focus on this for splitting arrays
Fz_net = Fz_net(bounds_mask); % clip edges where random things are happening

% Fy_net = F_net(:, 2);
Ftotal_net = vecnorm(F_net, 2, 2);
Ftotal_net = Ftotal_net(bounds_mask); % Fore-aft forces

contact_detect_thresh = 15; % > 15 Newtons is "in contact"
td_points = find(diff(sign(Fz_net - contact_detect_thresh)) > 0);  % identifies points where step lands
to_points = find(diff(sign(Fz_net - contact_detect_thresh)) < 0); % identifies points where step ends
num_strides = length(td_points);

%% store strides in matrix
dpps = 1500;
fz_strides = zeros(num_strides, dpps);  % pre allocate matrix
ftotal_strides = zeros(num_strides, dpps);
for i=1:num_strides
    stride_points = td_points(i):to_points(i);
    fz_strides(i, :) = interp1(Fz_net(stride_points), linspace(1, length(stride_points), dpps));
    ftotal_strides(i, :) = interp1(Ftotal_net(stride_points), linspace(1, length(stride_points), dpps));
end

%% find averages and plot strides
average_stride_fz = mean(fz_strides, 1);
average_stride_ftotal = mean(ftotal_strides, 1);

subplot(1, 2, 1);
title("Vertical forces");
hold on;
plot(fz_strides');
plot(average_stride_fz, "k-", LineWidth=5);
hold off;

subplot(1, 2, 2);
title("Net forces")
hold on;
plot(ftotal_strides');
plot(average_stride_ftotal, "k-", LineWidth=5);
hold off;

if save_result
    fz_saveFile = "avgForceCurves\" + curr_trial + "_fz_curve";
    save(fz_saveFile, "average_stride_fz");
    disp("saved fz data at: " + fz_saveFile);
    
    ftotal_saveFile = "avgForceCurves\" + curr_trial + "_ftotal_curve";
    save(ftotal_saveFile, "average_stride_ftotal");
    disp("saved ftotal data at: " + ftotal_saveFile);
end

rmpath(DATAPATH);