clc; clear; close all;

%% ==============================================================
%  Environment Initialisation
% ===============================================================

% Initialise ENV
env = common.make_env();

% Initialise Car
car = common.make_car(env.start);

% Canvas Initialisation
common.init_map(env, 1);

%% ==============================================================
%  RRT（Rapidly-exploring Random Tree） Path planning 
% ===============================================================

% Single RRT
[n_added, P_list] = path_planning.rrt.rrt(env);

% Build the Path
order_final_path = path_planning.rrt.rrt_path_build(n_added, P_list, env.goal);

% Canvas Initialisation
common.init_map(env, 2);

% re-draw the final rrt path on the second figure
plot(order_final_path(:,3), order_final_path(:,4), 'b-', 'LineWidth', 2);

% Optimize to find the short path
short_path = path_planning.rrt.rrt_shortcut_opt(order_final_path, env);

% Canvas Initialisation
common.init_map(env, 3);

% Dubins path planning
D_list = path_planning.dubins.dubins_path(short_path, env);

%% ==============================================================
%  Path following
% ===============================================================
path_following.carrot_chasing(D_list, car);



