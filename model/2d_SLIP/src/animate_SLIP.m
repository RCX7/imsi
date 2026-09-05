% animate_SLIP.m
% Roger Chen
% 09-05-2026
% Animates a SLIP model given a w_star

clc; clearvars -except w_star;

assert(exist('w_star', 'var'));
addpath('functions/sharedFuncs');

fps = 50;  % number of points

[N, ~] = simConstants();
[u, states, addDecs] = decToMats(w_star, 0);
[xs, xdots, zs, zdots, las] = extractStates(states);
[xf, ~, zf, ~] = flightKinematics(xs(end), xdots(end),...
    zs(end), zdots(end), 0:addDecs(1)/N:addDecs(2));

all_xs = [xs xf];
all_zs = [zs zf];

% prepare figure
figure('Color','w');
axis equal;
hold on;
yline(0, 'k-', "LineWidth", 3)

% determine limits with some padding
pad = 0.2;
xmin = min(all_xs) - pad;
xmax = max(all_xs) + pad;
zmin = -pad;
zmax = max(all_zs) + pad;
xlim([xmin xmax]);
ylim([zmin zmax]);
xlabel('x'); ylabel('z');
title('SLIP Animation');

% precompute trajectory classification: contact if distance to origin <= 1
r = sqrt(all_xs.^2 + all_zs.^2);
isContact = r <= 1;

% plot full trajectory background (light)
plot(all_xs, all_zs, '-', 'Color', [0.8 0.8 0.8]);

% overlay contact and aerial colored trajectory (solid)
plot(all_xs(isContact), all_zs(isContact), '.', 'Color', [1 0 0]);
plot(all_xs(~isContact), all_zs(~isContact), '.', 'Color', [0 0 1]);

% prepare animated objects
massRadius = 0.05 * max(1, range(xlim)); % small circle
hMass = plot(all_xs(1), all_zs(1), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 8);

% initial leg: if first frame contact use vector to mass, else set to length 1 at same angle as first frame
if isContact(1)
    legEnd = [all_xs(1), all_zs(1)];
else
    theta0 = atan2(all_zs(1), all_xs(1));
    legEnd = [cos(theta0), sin(theta0)]; % length 1
end
hLeg = plot([0, legEnd(1)], [0, legEnd(2)], '-k', 'LineWidth', 2);

% overlay the trajectory so far (progress)
hTrailContact = plot(nan, nan, '.', 'Color', [1 0 0]);
hTrailAerial = plot(nan, nan, '.', 'Color', [0 0 1]);

drawnow;

Nframes = numel(all_xs);
dt = 1 / fps;

% animate
% set up gif recording
gifFile = fullfile(pwd, 'animations/hop_SLIP_animation.gif');
frame = getframe(gcf);
im = frame2im(frame);
[imind, cm] = rgb2ind(im, 256);
imwrite(imind, cm, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', dt);

% during animation append frames inside loop (use a flag to skip rewriting first frame)
firstFrameWritten = true;
for k = 1:Nframes
    xk = all_xs(k);
    zk = all_zs(k);
    set(hMass, 'XData', xk, 'YData', zk);
    if isContact(k)
        % leg connects origin to actual mass position (may be <=1)
        legX = [0, xk];
        legZ = [0, zk];
    else
        % aerial: leg drawn at length 1 using initial angle to mass in first frame
        legX = [xk, xk - all_xs(1)];
        legZ = [zk, zk - all_zs(1)];
    end
    set(hLeg, 'XData', legX, 'YData', legZ);
    % update trail up to current frame
    set(hTrailContact, 'XData', all_xs(1:k).*isContact(1:k), 'YData', all_zs(1:k).*isContact(1:k));
    set(hTrailAerial, 'XData', all_xs(1:k).*~isContact(1:k), 'YData', all_zs(1:k).*~isContact(1:k));

    % capture current frame and write to gif
    frame = getframe(gcf);
    im = frame2im(frame);
    imind = rgb2ind(im, cm);
    imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', dt);

    drawnow;
    pause(dt);
end

rmpath('functions/sharedFuncs');