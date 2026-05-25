%% ==============================================================
%  Turn Left or Right Circle Centre Calculation
% ===============================================================
function [l_x_c, l_y_c, r_x_c, r_y_c] = calc_circle_centre(waypoint, r)

    x = waypoint(1);
    y = waypoint(2);
    th= waypoint(3);

    l_x_c = x + r * cos(th + pi/2);
    l_y_c = y + r * sin(th + pi/2);

    r_x_c = x + r * cos(th - pi/2);
    r_y_c = y + r * sin(th - pi/2);

end