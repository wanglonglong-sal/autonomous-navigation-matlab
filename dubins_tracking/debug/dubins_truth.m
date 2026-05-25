clc; clear; close all;

%% Dubins ground-truth path with MATLAB built-in APIs
% Pose format: [x, y, heading(rad)]
waypoint = [
    0,   10,  deg2rad(0);
    60,  60,  deg2rad(45);
    80,  120, deg2rad(30);
    150, 70,  deg2rad(-90);
    100, 30,  deg2rad(-120);
    50,  10,  deg2rad(-180);
];

r_radius = 5;  % minimum turning radius [m]

% MATLAB Navigation Toolbox Dubins connection object
conn = dubinsConnection('MinTurningRadius', r_radius);

n_wayp = size(waypoint, 1);
left_centers = zeros(n_wayp, 2);
right_centers = zeros(n_wayp, 2);

% Compute left/right candidate circle centers at each waypoint
for k = 1:n_wayp
    x = waypoint(k, 1);
    y = waypoint(k, 2);
    psi = waypoint(k, 3);

    left_centers(k, :) = [x - r_radius * sin(psi), y + r_radius * cos(psi)];
    right_centers(k, :) = [x + r_radius * sin(psi), y - r_radius * cos(psi)];
end

% Build full path by connecting adjacent waypoints with minimum-cost Dubins segment
full_path = waypoint(1, 1:2);
for k = 1:(n_wayp - 1)
    start_pose = waypoint(k, :);
    goal_pose = waypoint(k + 1, :);

    [allSegs, allCosts] = connect(conn, start_pose, goal_pose, 'PathSegments', 'all');
    [~, best_idx] = min(allCosts);
    best_seg = allSegs{best_idx};

    n_samples = max(120, ceil(best_seg.Length * 6));
    s = linspace(0, best_seg.Length, n_samples);
    poses = interpolate(best_seg, s);

    full_path = [full_path; poses(2:end, 1:2)]; %#ok<AGROW>
end

%% Plot
figure('Color', 'w', 'Name', 'Dubins Ground Truth');
hold on; grid on; axis equal;
set(gca, 'FontSize', 11, 'LineWidth', 1.0, 'GridAlpha', 0.18);

% Draw candidate left/right circles for each waypoint
th = linspace(0, 2*pi, 180);
hLeft = gobjects(1, 1);
hRight = gobjects(1, 1);
for k = 1:n_wayp
    lc = left_centers(k, :);
    rc = right_centers(k, :);

    h1 = plot(lc(1) + r_radius * cos(th), lc(2) + r_radius * sin(th), ...
        '--', 'Color', [0.90 0.25 0.25], 'LineWidth', 1.0);
    h2 = plot(rc(1) + r_radius * cos(th), rc(2) + r_radius * sin(th), ...
        '--', 'Color', [0.20 0.70 0.20], 'LineWidth', 1.0);

    if k == 1
        hLeft = h1;
        hRight = h2;
    end
end

% Dubins path
hPath = plot(full_path(:, 1), full_path(:, 2), '-', ...
    'Color', [0.10 0.35 0.95], 'LineWidth', 2.6);

% Waypoints and headings
hWay = plot(waypoint(:, 1), waypoint(:, 2), 'o', ...
    'Color', [0.10 0.10 0.10], 'MarkerFaceColor', [0.10 0.10 0.10], 'MarkerSize', 5);
arrow_len = 12;
hq = quiver(waypoint(:, 1), waypoint(:, 2), ...
    arrow_len * cos(waypoint(:, 3)), arrow_len * sin(waypoint(:, 3)), 0);
set(hq, 'Color', [0.15 0.15 0.15], 'LineWidth', 1.2, 'MaxHeadSize', 0.45, 'AutoScale', 'off');

% Circle centers
hLc = plot(left_centers(:, 1), left_centers(:, 2), '.', 'Color', [0.90 0.25 0.25], 'MarkerSize', 16);
hRc = plot(right_centers(:, 1), right_centers(:, 2), '.', 'Color', [0.20 0.70 0.20], 'MarkerSize', 16);

for k = 1:n_wayp
    text(waypoint(k, 1) + 1.2, waypoint(k, 2) + 1.2, sprintf('P%d', k), ...
        'Color', [0.1 0.1 0.1], 'FontSize', 9);
end

xlabel('X (m)');
ylabel('Y (m)');
title(sprintf('MATLAB Dubins Ground Truth (R_{min}=%.2f m)', r_radius));
legend([hLeft, hRight, hPath, hWay, hq, hLc, hRc], ...
    {'Left circles', 'Right circles', 'Dubins path', 'Waypoints', 'Heading', 'Left centers', 'Right centers'}, ...
    'Location', 'eastoutside');

% Tight axis limits around useful data
all_x = [full_path(:, 1); waypoint(:, 1); left_centers(:, 1); right_centers(:, 1)];
all_y = [full_path(:, 2); waypoint(:, 2); left_centers(:, 2); right_centers(:, 2)];
x_span = max(all_x) - min(all_x);
y_span = max(all_y) - min(all_y);
pad = max(20, 0.08 * max([x_span, y_span]));
xlim([min(all_x) - pad, max(all_x) + pad]);
ylim([min(all_y) - pad, max(all_y) + pad]);
