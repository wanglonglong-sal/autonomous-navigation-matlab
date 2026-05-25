%% ==============================================================
%  Draw A Arc
% ===============================================================
function draw_dubins_path(D, r, style)

    [idx, ...
    x_s, y_s, th_s, ...
    x_f, y_f, th_f, ...
    x_s_c, y_s_c, ...
    x_f_c, y_f_c, ...
    d, ...
    x_s_t, y_s_t, the_0_s_t, ...
    x_f_t, y_f_t, the_0_f_t, ...
    the_0_s_s, the_0_f_f, ...
    C1, C2] = data.parse_D_list(D);

    % Start Circle - rad path calculation
    the_0_s = atan2(y_s - y_s_c, x_s - x_s_c);
    the_0_t = atan2(y_s_t - y_s_c, x_s_t - x_s_c);
    if char(C1) == 'L'
        dth = wrapTo2Pi(the_0_t - the_0_s);
        % Draw the rad
        theta = linspace(the_0_s, the_0_s + dth, 100);
    else % 'R'
        dth = wrapTo2Pi(the_0_s - the_0_t);
        % Draw the rad
        theta = linspace(the_0_s, the_0_s - dth, 100);        
    end

    x = x_s_c + r * cos(theta);
    y = y_s_c + r * sin(theta);
    plot(x, y, style, 'LineWidth', 2);

    % Draw S between tangent points
    draw_point(x_s_t, y_s_t, 'ro');
    draw_point(x_f_t, y_f_t, 'ro');
    plot([x_s_t, x_f_t], [y_s_t, y_f_t], style, "LineWidth", 2);


    % End Circle - rad path calculation
    the_0_enter = atan2(y_f_t - y_f_c, x_f_t - x_f_c);
    the_0_final = atan2(y_f - y_f_c, x_f - x_f_c);

    if char(C2) == 'L'
        dth_f = wrapTo2Pi(the_0_final - the_0_enter);
        theta_f = linspace(the_0_enter, the_0_enter + dth_f, 100);
    else % 'R'
        dth_f = wrapTo2Pi(the_0_enter - the_0_final);
        theta_f = linspace(the_0_enter, the_0_enter - dth_f, 100);    
    end    

    xx = x_f_c + r * cos(theta_f);
    yy = y_f_c + r * sin(theta_f);
    plot(xx, yy, style, 'LineWidth', 2);

end

%% ==============================================================
%  Draw A Point
% ===============================================================
function draw_point(x, y, style)

    plot(x, y, style, 'MarkerSize', 4, 'LineWidth', 1, 'MarkerFaceColor', 'y');

end