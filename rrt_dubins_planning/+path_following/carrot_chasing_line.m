%% ==============================================================
%  Path following
% ===============================================================
function carrot_chasing(start, order_final_path)

    % Car Initialisation
    car = common.make_car(start);

    % Car initial position
    car_x = car.x;
    car_y = car.y;
    car_th = car.th;

    % Runing time
    T = 1000; %[s]
    % Timestep
    dt = 0.5;
    % Timestep number
    N_t = T/dt;

    % Carrot Chasing Algorithm Parameter
    look_ahead = 10; %[m]
    % Control Gain
    gain = 2;       %[/m]
    % Distance thredshold
    eps = 5;        %[m]
    % Path idx
    path_idx = 1;
    % Size of final path
    n_final_path = size(order_final_path,1);

    % Animation - Car movement
    car_p = plot(car_x, car_y, 'ko','MarkerSize',10,'LineWidth',2);
    % Animation - Car heading
    arrow = quiver(car_x, car_y,car_x + car.r * cos(car_th), car_y + car.r * sin(car_th));
    % Animation - trajectory
    traj = plot(nan, nan, 'y-', 'LineWidth', 2);
    % History of position
    x_hist = nan(1, N_t);
    y_hist = nan(1, N_t);

    % Path Following Main Process
    for i = 1:N_t

        % Update trajectory history
        x_hist(i) = car_x;
        y_hist(i) = car_y;

        % Calculate unit normal vector
        d_v = order_final_path(path_idx + 1,3:4) - order_final_path(path_idx,3:4);
        d = max(norm(d_v), 1e-6);
        d_th = d_v/d;

        % Calculate progress position by using projection
        progress = dot([car_x, car_y] - order_final_path(path_idx,3:4), d_th);
        progress = min(max(progress, 0), d);     

        % Calculate VTP as carrot
        carrot = order_final_path(path_idx,3:4) + min(progress + look_ahead, d) * d_th; 

        % Calculate the difference between car heading and carrot
        the_car_2_caro = atan2(carrot(2) - car_y, carrot(1) - car_x);
        the_diff_car_caro = wrapToPi(the_car_2_caro - car_th);

        % Control 
        ang_vel = gain * the_diff_car_caro * car.v;
        ang_vel = max(min(ang_vel, car.w_max), -car.w_max);  

        % Moving
        car_x = car_x + car.v*cos(car_th)*dt;
        car_y = car_y + car.v*sin(car_th)*dt;
        car_th = car_th + ang_vel * dt;

        % Animation
        set(car_p, 'XData',car_x, 'YData', car_y);
        set(arrow, 'XData',car_x, 'YData', car_y, 'UData', 2*cos(car_th), 'VData', 2*sin(car_th));
        set(traj, 'XData', x_hist(1:i), 'YData', y_hist(1:i));
        drawnow

        % Update State
        if (d - progress) < eps 
            path_idx = path_idx + 1;
        end
        if path_idx >= n_final_path
            break;
        end

    end
end