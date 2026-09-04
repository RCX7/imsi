% avgForceCurves.m
% Roger Chen
% 2027-07-30
% Given force data, finds horizontal, vertical, and total force components
% for that data. Pulls from exp_data folder.

clc; clearvars; close all;

subject = "S05";
save_result = false;

DATAPATH = "C:\Users\roger\imsi\exp_data\" + subject + "Data_unc\ForceData";
disp("Data source: " + DATAPATH);
addpath(DATAPATH);

%% settings
gait='r';
% trials = {'hop_1ms_unc_30sec'; 'run_1ms_unc_1min'};
trials = {'hop_2ms'; 'run_2ms'};
if gait == 'r', curr_trial = trials{2}; else, curr_trial = trials{1}; end
g = 9.81;       % gravity
fs = 2000;      % sampling frequency
tsamp = 1 / fs; % sampling period

%% import and extract the data
force_data = importdata([curr_trial '.mot']);  % for loading from S01
% switch gait
%     case 'r'
%         bounds_mask = 1:19520;     % needs to be manually tuned.
%     case 'h'
%         bounds_mask = 380:19700;
%         % bounds_mask = 0:19500;
% end

F_left = force_data.data(:, [2, 4, 3]);
F_right = force_data.data(:, [11, 13, 12]);
F_total = F_left + F_right;
Fz_total = F_total(:,3);

lb = find(Fz_total <= 0, 1, 'first');
ub = find(Fz_total <= 0, 1, 'last');
bounds_mask = lb:ub;

F_left = F_left(bounds_mask, :); F_right = F_right(bounds_mask, :);
switch gait
    case 'r'
        F_net = F_right;
    case 'h'
        F_net = F_left + F_right;
end
Fz_net = F_net(:,3);

t = force_data.data(:,1);
t = t(bounds_mask);
t_total = t(end) - t(1);


%% find stride times
% Fy_net = F_net(:, 2);
Ftotal_net = vecnorm(F_net, 2, 2);

contact_detect_thresh = 10; % > 10 Newtons is "in contact"
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

tstance = to_points - td_points;
tstance_avg = mean(tstance) * tsamp; 
tflight = td_points(2:end) - to_points(1:end-1);
tflight_avg = mean(tflight) * tsamp;

tstride = diff(to_points);
tstride_avg = mean(tstride) * tsamp;

if gait == 'r', tstride_avg = tstride_avg / 2; end

stride_freq = 1 / (mean(diff(to_points)) * tsamp);
ttotals = diff(to_points);
dutyfac = mean(tstance(1:numel(ttotals)) ./ ttotals);

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


curve_timing = "full_stride";
if save_result
    fz_saveFile = "avgForceCurves\" + subject + "\" + curve_timing + "\" + subject + "_" + curr_trial + "_fz_curve";
    save(fz_saveFile, "average_stride_fz", "tstance_avg", "tflight_avg", "tstride_avg");
    disp("saved fz data at: " + fz_saveFile);
    
    ftotal_saveFile = "avgForceCurves\" + subject + "\" + curve_timing + "\" + subject + "_" + curr_trial + "_ftotal_curve";
    save(ftotal_saveFile, "average_stride_ftotal", "tstance_avg", "tflight_avg", "tstride_avg");
    disp("saved ftotal data at: " + ftotal_saveFile);
end

rmpath(DATAPATH);