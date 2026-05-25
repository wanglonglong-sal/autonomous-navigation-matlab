function [n_added, P_list] = rrt(env)

%% ==============================================================
%  Algorithmic Parameter Settings
% ===============================================================
% Node toal number
n_node = 1000;

% Step size
step_size = 15; %[m]

% node idx in tree
n_added = 1;

% Point list
P_list = nan(n_node, 4);
% 1: parent node; 2: current node; 3-4 current position
P_list(n_added,:) = [0, 1, env.start];

% Distance list
d_list = nan(n_node, 1);

%% ==============================================================
%  Main Process
% ===============================================================
for i = 1:n_node

    % Initialize distance list
    d_list = inf(n_added, 1);

    % Generate random points
    x_rand = env.x_min + rand * (env.x_max - env.x_min);
    y_rand = env.y_min + rand * (env.y_max - env.y_min);
    p_rand = [x_rand, y_rand];

    % Draw rand point in each round
    plot(x_rand, y_rand, 'ro', 'LineWidth',1, 'MarkerSize',1);
    % Showing the round number in title part
    title(sprintf("RRT Nodes: %d", n_added));    

    % Find the nearest point - Calculate the distance between random point and each point
    for j = 1:n_added
        % Distance
        d = norm(p_rand - P_list(j,3:4));
        d_list(j) = d;          
    end

    % Get the min distance in the d_list
    [m, idx] = min(d_list);
    % Get the nearest point
    P_nearest = P_list(idx,3:4);
    % Direction Vector
    d_v = p_rand - P_nearest;
    % Distance
    d = norm(p_rand - P_nearest);
    % Moving direction
    th_d = d_v / d;
    % Moving distance
    p_new = P_nearest + step_size * th_d;

    % Collision with obstacles - in or on
    result = path_planning.rrt.chk_collision([P_nearest; p_new], env.poly);
    % No collision then update P_list
    if result == 0
        % Save the new point to the list
        P_list(n_added + 1,:) = [idx, n_added + 1, p_new];  
        n_added = n_added + 1; 
        % Draw the path
        plot([P_list(idx,3:3) p_new(1)], [P_list(idx, 4:4) p_new(2)], 'r-','LineWidth',1.5);        
        % Goal Reaching
        d_g = norm(p_new - env.goal);
        if d_g < env.goal_radius
            break;
        end        
    end

    drawnow;
end
end