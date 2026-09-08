% multiVarCOTSweeps.m
% Roger Chen
% 2026-09-07
% Fits spring constant (k), damping constant(c), and inertia (I) based on
% MSE of force curves.

clc; clearvars; %close all;

% mdl_src_path = 'C:\Users\azizi\Downloads\rogerc8_imsi_work\imsi\model\2d_SLIP\src';
% mdl_src_path = 'C:\Users\roger\imsi\model\2d_SLIP\src';
mdl_src_path = 'C:\Users\Public\Documents\Roger\imsi\model\2d_SLIP\src';
mdl_func_path = fullfile(mdl_src_path, "functions");
shared_func_path = fullfile(mdl_func_path, "sharedFuncs");
singleOpt_path = fullfile(mdl_func_path, "singleOpt");
multiOpt_path = fullfile(mdl_func_path, "multiOpt");
warmStartTemp_path = fullfile(mdl_src_path, "warmStartTemplates");

addpath(shared_func_path);
addpath(warmStartTemp_path);

[num_mdl_pts, ~] = simConstants();
% like physConstants but shouldn't be tied to a particular human
g_uN = 9.81;
l_uN = 1.15;   % length of leg in meters
t_sim = sqrt(l_uN / g_uN);
v_scale = (l_uN / t_sim);

target_speed_uN = 2;

ks = 20:2.5:40;
cs = 0.0025:0.0025:0.03;
speeds = 1:8;  % 1-8m/s
I = 0.01;

[k_mesh, c_mesh, speed_mesh] = meshgrid(ks, cs, speeds);
k_vec = k_mesh(:);
c_vec = c_mesh(:);
speed_vec = speed_mesh(:);


%% set up output (flattened for parallelization)
n_sims = numel(k_vec);
COT_diff_landscape = zeros(n_sims, 1);

% parallelize optimizations
addpath(multiOpt_path);
% cluster = parcluster('local'); 
% cluster.NumWorkers = 24; 
% saveProfile(cluster); 
% parpool(24); 
parfor i=1:n_sims
    k_run = k_vec(i);
    k_hop = 2 * k_run;
    c_run = c_vec(i);
    c_hop = 2 * c_run;

    I_run = I;
    I_hop = 2 * I;
    target_speed_uN = speed_vec(i);
    target_speed = target_speed_uN * (l_uN / t_sim);
    
    
    [run_COT, ~, ~, ~, ~, ~] = ...
                robustMinCOT('r', target_speed, 0, k_run, c_run, I_run, 0);
    [hop_COT, ~, ~, ~, ~, ~] = ...
                robustMinCOT('h', target_speed, 0, k_hop, c_hop, I_hop, 0);

    COT_diff = hop_COT - run_COT;
    COT_diff_landscape(i) = COT_diff;

end

%% reshape landscape and plot slices
COT_diff_landscape = reshape(COT_diff_landscape, size(k_mesh));
COT_diff_landscape(COT_diff_landscape == 0) = max(COT_diff_landscape, [], "all");  % not sure the replacement here..
COT_diff_landscape = clip(COT_diff_landscape, -1, 1);  % may not be necessary

num_subplots = numel(speeds);
% num_cols = 4;
% num_rows = ceil(num_subplots / num_cols);

for i=1:num_subplots
    curr_slice = COT_diff_landscape(:,:,i);
    curr_speed = speeds(i);
    
    figure;
    % subplot(num_rows, num_cols, i);
    surf(ks, cs, curr_slice);
    colormap jet;
    title(sprintf('COT Diff Landscape Slice for speed = %.5f', curr_speed));
    xlabel('Spring Constant (k)');
    ylabel('Damping Constant (c)');
    zlabel('COT diff (hop - run)');
end

%% save MSE landscape as gif
gif_filename = "animations/COT_landscape_slices.gif";
fig = figure('Visible','off');
for i=1:num_subplots
    curr_slice = COT_diff_landscape(:,:,i);
    curr_speed = speeds(i);
    clf(fig);
    surf(ks, cs, curr_slice);
    view(0, 90);
    shading flat;
    colormap jet;
    colorbar;
    %caxis([-2, 2]);
    title(sprintf('COT diff landscape Slice for speed = %.5f', curr_speed));
    xlabel('Spring Constant (k)');
    ylabel('Damping Constant (c)');
    zlabel('COT diff (hop - run)');
    drawnow;
    frame = getframe(fig);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);
    if i == 1
        imwrite(A,map,gif_filename,'gif','LoopCount',Inf,'DelayTime',0.2);
    else
        imwrite(A,map,gif_filename,'gif','WriteMode','append','DelayTime',0.2);
    end
end
close(fig);

rmpath(multiOpt_path);
rmpath(shared_func_path);
rmpath(warmStartTemp_path);
