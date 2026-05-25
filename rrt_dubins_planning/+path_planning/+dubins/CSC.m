%% ==============================================================
%  CSC Distance Calculation
% ===============================================================
function [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = CSC(C1, C2, waypoint1, waypoint2, cc1, cc2, r)

    % Start Circle - Initial waypoint state
    w_x_s = waypoint1(1,1);
    w_y_s = waypoint1(1,2);
    w_th_s = waypoint1(1,3);
   
    % End Circle - Initial waypoint state
    w_x_f = waypoint2(1,1);
    w_y_f = waypoint2(1,2);
    w_th_f = waypoint2(1,3);

    % Start Circle - Initial circle centre
    if C1 == 'L'
        c_x_s = cc1(1);
        c_y_s = cc1(2);
    elseif C1 == 'R'
        c_x_s = cc1(3);
        c_y_s = cc1(4);
    else
        error("CSC C1 must be 'L' or 'R'");       
    end
    % End Circle - Initial circle centre
    if C2 == 'L'    
        c_x_f = cc2(1);
        c_y_f = cc2(2);
    elseif C2 == 'R'
        c_x_f = cc2(3);
        c_y_f = cc2(4);
    else
        error("CSC C2 must be 'L' or 'R'");          
    end

    % Center-to-center distance calculation
    dx = c_x_f - c_x_s;
    dy = c_y_f - c_y_s;
    d_cc = norm([dx, dy]);
    % center-to-center angle
    the_cc = atan2(dy, dx);
    % OUTER tangent
    if C1 == C2 % LSL, RSR
        if C1 == 'L'
            the_tan = the_cc - pi/2;
            turn_sign_s = +1;
            turn_sign_f = +1;
        else 
            the_tan = the_cc + pi/2;
            turn_sign_s = -1;
            turn_sign_f = -1;
        end
        % tangent point of start circle
        t_x_s = c_x_s + r * cos(the_tan);
        t_y_s = c_y_s + r * sin(the_tan);
        t_s = [t_x_s, t_y_s];

        % tangent point of end circle
        t_x_f = c_x_f + r * cos(the_tan);
        t_y_f = c_y_f + r * sin(the_tan);
        t_f = [t_x_f, t_y_f];    
        % distance of the external tangent
        d_t = norm([t_x_f - t_x_s, t_y_f - t_y_s]);
        % Sanity Check - in LSL case d_cc should equal d_t
        if abs(d_cc - d_t) > 1e-6
            warning("CSC outer: d_cc != d_t (%.6f)", abs(d_cc - d_t));
        end

        % start circle - arc length calculation
        the_0_s_s = atan2(w_y_s - c_y_s, w_x_s - c_x_s);
        the_0_s_t = atan2(t_y_s - c_y_s, t_x_s - c_x_s);
        dthe_s_s_t = wrapTo2Pi(turn_sign_s * (the_0_s_t - the_0_s_s));

        L_arc_s = r * dthe_s_s_t;

        % end circle - arc length calculation
        the_0_f_f = atan2(w_y_f - c_y_f, w_x_f - c_x_f);
        the_0_f_t = atan2(t_y_f - c_y_f, t_x_f - c_x_f);
        dthe_f_t_f = wrapTo2Pi(turn_sign_f * (the_0_f_f - the_0_f_t));

        L_arc_f = r * dthe_f_t_f;

        % LSL length d
        d = L_arc_s + d_t + L_arc_f;            
    % INNER tangent
    else        
        % No answer in this case
        if d_cc < 2*r
            d = inf; t_s = [nan nan]; t_f = [nan nan];
            return;
        end      
        
        alpha = acos(2*r / d_cc);
        
        if C1 == 'L' && C2 == 'R'
            theta = the_cc - alpha;
        else
            theta = the_cc + alpha;
        end

        if C1=='L' && C2 =='R'
            the_tan_s = theta;
            the_tan_f = theta + pi;
            turn_sign_s = +1;
            turn_sign_f = -1;            
        elseif C1=='R' && C2 =='L'
            the_tan_s = theta;
            the_tan_f = theta + pi;   
            turn_sign_s = -1;
            turn_sign_f = +1;                         
        end

        % tangent point of start circle
        t_x_s = c_x_s + r * cos(the_tan_s);
        t_y_s = c_y_s + r * sin(the_tan_s);
        t_s = [t_x_s, t_y_s];

        % tangent point of end circle
        t_x_f = c_x_f + r * cos(the_tan_f);
        t_y_f = c_y_f + r * sin(the_tan_f);
        t_f = [t_x_f, t_y_f];
        
        % Sanity Check - tangent line perpendicular at r
        v_line = [t_x_f - t_x_s, t_y_f - t_y_s];
        v_rad_s = [t_x_s - c_x_s, t_y_s - c_y_s];
        if abs(dot(v_line, v_rad_s)) > 1e-6, warning('INNER tangent not perpendicular at start'); end

        % distance of the external tangent
        d_t = norm([t_x_f - t_x_s, t_y_f - t_y_s]);

        % start circle - arc length calculation
        the_0_s_s = atan2(w_y_s - c_y_s, w_x_s - c_x_s);
        the_0_s_t = atan2(t_y_s - c_y_s, t_x_s - c_x_s);
        dthe_s_s_t = wrapTo2Pi(turn_sign_s * (the_0_s_t - the_0_s_s));

        L_arc_s = r * dthe_s_s_t;

        % end circle - arc length calculation
        the_0_f_f = atan2(w_y_f - c_y_f, w_x_f - c_x_f);
        the_0_f_t = atan2(t_y_f - c_y_f, t_x_f - c_x_f);
        dthe_f_t_f = wrapTo2Pi(turn_sign_f * (the_0_f_f - the_0_f_t));

        L_arc_f = r * dthe_f_t_f;

        % LSL length d
        d = L_arc_s + d_t + L_arc_f; 
        
    end
end