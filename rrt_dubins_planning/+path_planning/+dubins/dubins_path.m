function D_list = dubins_path(short_path, env)

% Optimize to build Dubins path by short path
n = size(short_path, 1);
waypoint = zeros(n,3);
waypoint(:,1:2) = short_path(:,3:4); 
for i = 1:n
    if i == 1
        waypoint(i,3) = deg2rad(0);
    elseif i == n
        waypoint(i,3) = deg2rad(90);
    else
        p_prev = waypoint(i-1, 1:2);
        p_next = waypoint(i+1, 1:2);
        waypoint(i,3) = atan2(p_next(2)-p_prev(2), p_next(1)-p_prev(1));
    end
end

% Number of waypoint
n_wayp = size(waypoint, 1);
% Circle centre list to save turn lef or right circle centre position
cc_list = nan(n_wayp, 4);
% Distance list 
D_list = nan(n_wayp - 1, 22);

car = common.make_car(env.start);

% Draw all waypoint initial pose and heading
for i = 1:n_wayp
    draw_waypoints(waypoint(i,:));
end

% Left and Right Circle center calculation
for i = 1:n_wayp
    % Depending on waypoint state and return radius to compute
    [l_x_c, l_y_c, r_x_c, r_y_c] =  path_planning.dubins.calc_circle_centre(waypoint(i,:), car.r);
    cc_list(i,:) =  [l_x_c, l_y_c, r_x_c, r_y_c];
    % Draw left with blue color
    draw_circle(l_x_c, l_y_c, car.r, 'r--');
    % Draw right circle with green color
    draw_circle(r_x_c, r_y_c, car.r, 'g--');
end

for i = 1:n_wayp - 1

    % D_list(i,:) = path_planning.dubins_path_planning(waypoint(i,:), waypoint(i+1,:), cc_list(i,:), cc_list(i+1,:), r_radius);
    % Initialised candidate list
    candi_list = nan(4, 9);
    % LSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.dubins.CSC('L','L',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), car.r);
    candi_list(1,:) = [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];
    % RSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.dubins.CSC('R','R',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), car.r);
    candi_list(2,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];    
    % LSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.dubins.CSC('L','R',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), car.r);
    candi_list(3,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];  
    % RSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.dubins.CSC('R','L',waypoint(i,:), waypoint(i+1,:), cc_list(i,:),cc_list(i+1,:), car.r);
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
    path_planning.dubins.draw_dubins_path(D_list(i,:), car.r, 'b-');
end

end

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