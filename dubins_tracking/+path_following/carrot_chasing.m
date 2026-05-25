function carrot_chasing(D_list, r_radius)
    %% ==============================================================
    %  Variable Initialisation
    % ===============================================================
    % UGV velocity
    v = 5; %[m/s]
    % Total time
    T = 1000; %[s]
    % Timestep
    dt = 0.05; %[s]
    % Max lateral acceleration
    u_max = 5; %[m/s^2]
    % Max augular velocity
    ang_v_max = u_max / v;
    % Count Time
    N_t = T/dt;
    % Car Movement Initialisation
    car_x = -30;
    car_y = 30;
    car_psi = deg2rad(180);    
    % Car movement
    car = plot(nan, nan, 'bo', 'MarkerSize', 8);
    arrow = quiver(0,0,0,0,'r');
    % Car trajectory
    traj = plot(nan, nan, 'r-');
    x_hist = nan(1, N_t);
    y_hist = nan(1, N_t);
    % Gain
    kpsi = 5.0;
    % Look ahead meter for line
    delta_lahead = 5; %[m]
    % Look ahead for arc
    lamuda_lahead = 0.5; %[rad]
    % Main Path segment state
    seg = 1;
    % Sub-Path segment state
    path_idx = 2;
    % Distance threshold
    eps = 1; %[m]

    % --------  Initial State Parse  -------- 
    [idx, ...
    x_s, y_s, th_s, ...
    x_f, y_f, th_f, ...
    x_s_c, y_s_c, ...
    x_f_c, y_f_c, ...
    d, ...
    x_s_t, y_s_t, the_0_s_t, ...
    x_f_t, y_f_t, the_0_f_t, ...
    the_0_s_s, the_0_f_f, ...
    C1, C2] = data.parse_D_list(D_list(seg,:));

    % Draw the first part straight path
    plot([x_s_t x_f_t],[y_s_t y_f_t], 'y--', 'LineWidth', 1.5);    

    %% ==============================================================
    %  Main Loop (Carror & Chasing)
    % ===============================================================
    for i = 1:N_t
        
        % save trajectory history
        x_hist(i) = car_x;
        y_hist(i) = car_y;

        % Depending on distance between car and start circle tagent point to ensure the path segment idx allocation
        if path_idx == 1 && norm([car_x - x_s_t, car_y - y_s_t]) < eps
            path_idx = 2;
        elseif path_idx == 2 && norm([car_x - x_f_t, car_y - y_f_t]) < eps
            path_idx = 3;
        end

        % Sub-Path segment idx 1: in start circle -> 2: in straight line -> 3: in final circle; the idx can not being backward deliver
        if path_idx == 1
            [carrot, th_carrot, sgn] = chasing_arc( x_s_c, y_s_c, the_0_s_s, the_0_s_t, C1, r_radius, car_x, car_y, lamuda_lahead);
            % desired heading calculation
            psi_tan = wrapToPi(th_carrot + sgn*pi/2);
            psi_los = atan2(carrot(2)-car_y, carrot(1)-car_x);
            psi_d = wrapToPi(0.7*psi_tan + 0.3*psi_los);            
        elseif path_idx == 2
            carrot = chasing_line([x_s_t, y_s_t], [x_f_t, y_f_t], car_x, car_y, delta_lahead, i);
            psi_d = atan2(carrot(2)-car_y, carrot(1)-car_x);
        else
            [carrot, th_carrot, sgn] = chasing_arc(x_f_c, y_f_c, the_0_f_t, the_0_f_f, C2, r_radius, car_x, car_y, lamuda_lahead);
            psi_tan = wrapToPi(th_carrot + sgn*pi/2);
            psi_los = atan2(carrot(2)-car_y, carrot(1)-car_x);
            psi_d = wrapToPi(0.7*psi_tan + 0.3*psi_los);            
        end
        
        % heading control 
        epsi = wrapToPi(psi_d - car_psi);
        % Calculate angular velocity (u) - it is not control law , it is a rate control by gain
        % formula: κ=ω/v -> κ=kψ​eψ = ω=kψ​eψ​v; 
        u = kpsi * epsi * v;
        u = max(min(u, ang_v_max), -ang_v_max);

        % dynamics
        car_x = car_x + v * cos(car_psi) * dt;
        car_y = car_y + v * sin(car_psi) * dt;
        % car_psi = car_psi + (u/v) * dt;
        car_psi = car_psi + u * dt;

        % draw the animation of car movement
        set(car, 'XData', car_x, 'YData', car_y);
        set(arrow, 'XData', car_x, 'YData', car_y, 'UData', 2*cos(car_psi), 'VData', 2*sin(car_psi));
        set(traj, 'XData', x_hist(1:i), 'YData', y_hist(1:i));
        drawnow;

        % Main Path State Check: the car is close to final point, state change to next part
        if norm([car_x - x_f, car_y - y_f]) < eps
            if seg < size(D_list, 1)
                % Next State Update        
                seg = seg + 1;
                path_idx = 1;
                % --------  Update Path Segment Parameters  -------- 
                [idx, ...
                x_s, y_s, th_s, ...
                x_f, y_f, th_f, ...
                x_s_c, y_s_c, ...
                x_f_c, y_f_c, ...
                d, ...
                x_s_t, y_s_t, the_0_s_t, ...
                x_f_t, y_f_t, the_0_f_t, ...
                the_0_s_s, the_0_f_f, ...
                C1, C2] = data.parse_D_list(D_list(seg,:));     
                % draw the straight line as path
                plot([x_s_t x_f_t],[y_s_t y_f_t], 'y--', 'LineWidth', 1.5);                                
            else
                break;
            end
        end
        
    end

end

%% ==============================================================
%  Get Carrot in ARC
% ===============================================================
function [carrot, th_carrot, sgn] = chasing_arc(cx , cy, th_start, th_end, C, r, car_x, car_y, look_ahead)

    % Arc length calculation, sgn showing the turning direction, detal showing the turning length 
    if char(C) == 'L'
        arc_length = wrapTo2Pi(th_end - th_start);
        sgn = +1;
    else
        arc_length = wrapTo2Pi(th_start - th_end);
        sgn = -1;            
    end

    % Projection car position to circle
    th_car = atan2(car_y - cy, car_x - cx);

    % Car progress evaluation: Calculate the distance between start and final
    if char(C) == 'L'
        car_progress = wrapTo2Pi(th_car - th_start);
    else
        car_progress = wrapTo2Pi(th_start - th_car);
    end

    % Just in case to prevent progress greater than total distance of arc
    car_progress = min(max(car_progress, 0), arc_length);

    % carrot length in arc - Δθ=s/r​
    % dth_la = look_ahead / r;
    dth_la = look_ahead; 

    % carrot theta respect to progress
    prog2 = min(car_progress + dth_la, arc_length);

    % carrot theta respect to zero
    th_carrot = th_start + sgn * prog2; 
    
    % carrot position
    carrot = [cx + r*cos(th_carrot), cy + r*sin(th_carrot)];
  

    %plot(carrot(1), carrot(2), 'o', 'Color', [1 0.5 0], "MarkerSize",5, "LineWidth",5);

end

%% ==============================================================
%  Get Carrot in Line
% ===============================================================
function carrot = chasing_line(t_entry, t_exit, car_x, car_y, look_ahead, i)


    % distance of tangent line
    dtx = t_exit(1) - t_entry(1);
    dty = t_exit(2) - t_entry(2);
    d_t = norm([dtx, dty]);

    % direction of tangent line （unit tangent vector）
    utv = [dtx, dty] / d_t;

    % progress of the car in the tangent path - calculate the projection of car position
    car_progress = dot([car_x, car_y] - t_entry, utv);
    car_progress = min(max(car_progress, 0), d_t);

    % carrot position
    carrot = t_entry + min(car_progress + look_ahead, d_t) * utv;

    if mod(i, 50) == 0
        plot(carrot(1), carrot(2), 'o', 'Color', [1 0.5 0], "MarkerSize",5, "LineWidth",5);
    end
    % plot(carrot(1), carrot(2), 'ro', "MarkerSize",5, "LineWidth",5);


end

