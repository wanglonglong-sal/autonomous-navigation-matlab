clc; clear; close all;

%% ==============================================================
%  Variable Initialisation
% ===============================================================

% Waypoint list 6*3 matrix
waypoint = [
    [0,   10,  deg2rad(0)];
    [60,  60,  deg2rad(45)];
    [80,  120, deg2rad(30)];
    [150, 70,  deg2rad(-90)];
    [100, 30,  deg2rad(-120)];
    [50,  10,  deg2rad(-180)];
]; % x_position in [m], y_position in [m], heading in [rad]
% Number of waypoint
n_wayp = size(waypoint, 1);
% return radius
r_radius = 5; %[m]
% Circle centre list to save turn lef or right circle centre position
cc_list = nan(n_wayp, 4);
% Distance list 
D_list = nan(n_wayp - 1, 22);

%% ==============================================================
%  Canvas Initialisation
% ===============================================================
figure; grid on; hold on;
axis equal;

% Draw all waypoint initial pose and heading
for i = 1:n_wayp
    draw_waypoints(waypoint(i,:));
end

%% ==============================================================
%  Dubins Path Calculation
% ===============================================================

% Left and Right Circle center calculation
for i = 1:n_wayp
    % Depending on waypoint state and return radius to compute
    [l_x_c, l_y_c, r_x_c, r_y_c] =  path_planning.calc_circle_centre(waypoint(i,:), r_radius);
    cc_list(i,:) =  [l_x_c, l_y_c, r_x_c, r_y_c];
    % Draw left with blue color
    draw_circle(l_x_c, l_y_c, r_radius, 'r--');
    % Draw right circle with green color
    draw_circle(r_x_c, r_y_c, r_radius, 'g--');
end

% Distance Calculation in LSL, RSR, LSR, RSL
% for i = 2:2
for i = 1:n_wayp - 1

    % D_list(i,:) = path_planning.dubins_path_planning(waypoint(i,:), waypoint(i+1,:), cc_list(i,:), cc_list(i+1,:), r_radius);
    % Initialised candidate list
    candi_list = nan(4, 9);
    % LSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('L','L',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), r_radius);
    candi_list(1,:) = [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];
    % RSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('R','R',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), r_radius);
    candi_list(2,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];    
    % LSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('L','R',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), r_radius);
    candi_list(3,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];  
    % RSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('R','L',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), r_radius);
    candi_list(4,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];   

    % Gain the minimun value
    vals = candi_list(:,1);
    vals(~isfinite(vals)) = nan;
    [min_val, idx] = min(vals, [], "omitnan");

    % Sort of Dataset
    switch(idx)
        case 1
            C1 = 76; %'L'
            C2 = 76; %'L'
            cc_s = cc_list(i,1:2);
            cc_f = cc_list(i+1,1:2);
        case 2
            C1 = 82; %'R'
            C2 = 82; %'R'
            cc_s = cc_list(i,3:4);
            cc_f = cc_list(i+1,3:4);
        case 3
            C1 = 76; %'L'
            C2 = 82; %'R'
            cc_s = cc_list(i,1:2);
            cc_f = cc_list(i+1,3:4);
        case 4
            C1 = 82; %'R'
            C2 = 76; %'L'
            cc_s = cc_list(i,3:4);
            cc_f = cc_list(i+1,1:2); 
    end

    D_list(i,:) = [idx, waypoint(i,:), waypoint(i+1,:), cc_s, cc_f, candi_list(idx,:), C1, C2];
    % D_list 1:idx, 2-4: x_s,y_s,theta_s; 5-7: x_f, y_f, theta_f; 8-9:cc_s_x, cc_s_y;
    %        10-11: cc_f_x, cc_f_y; 12: path distance; 13-14:Tangent Point S x, y; 15: Tangent angle the_0_s_t    
    %        16-17: Tangent Point F x,y; 18: Tangent angle the_0_f_t ; 21-22:  C1(L or R),  C2(L or R)
    path_planning.draw_dubins_path(D_list(i,:), r_radius, 'b-');
end
%% ==============================================================
%  Carrot Chasing
% ===============================================================

path_following.carrot_chasing(D_list, r_radius);


%% ==============================================================
%  Draw Left and Right Circle
% ===============================================================
function draw_circle(cx, cy, r, style)

    % Generate 200 points from 0 to 2pi
    ang = linspace(0, 2*pi, 200);

    x = cx + r * cos(ang);
    y = cy + r * sin(ang);

    plot(x, y, style, 'LineWidth', 1);

end

%% ==============================================================
%  Draw Waypoint Position and Heading
% ===============================================================
function draw_waypoints(waypoint)

    % Define the length of heading arrow
    L = 5;

    x = waypoint(1);
    y = waypoint(2);
    th= waypoint(3);
    % Draw the specific position
    plot(x, y, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
    % Draw the heading
    quiver(x, y, L*cos(th), L*sin(th), 0, 'r', 'LineWidth', 2);

end