function car = make_car(start)

% Car initial position
car.x = start(:,1);     %[m]
car.y = start(:,2);     %[m]

% Car initial heading
car.th = deg2rad(0);   %[rad]

% Car Initial State
car.state = [car.x, car.y, car.th];

% Car turning state
car.turning = false;

car.turning_cool = 0;
car.ep_th = 0.0;

% Car minimum return radius
car.r = 5;              %[m]

% Linear velocity
car.v = 5.0;                  %[m/s]

% Constant Angular velocity
car.w = 0.1;                  %[rad/s]

% Maximum Linear velocity 
car.v_max = 5;             %[m/s]

% Maximum angular velocity
car.w_max = 1.0;            %[rad/s]

% Maximum lateral acceleration 
car.u_max = 1.5;            %[m/s^2]

% sensing radius
car.sensor_r = 100;          %[m]

% Car Animation
car.pose = plot(nan, nan, 'bo', 'MarkerSize', 12, 'MarkerFaceColor', [0 0.6 1], 'MarkerEdgeColor', 'k', 'LineWidth', 3);
car.heading = quiver(0, 0, 0, 0,'r', 'LineWidth',2);
% Car trajectory
car.traj = plot(nan, nan, 'b--', 'LineWidth',1.5);
car.perception = plot(nan, nan, 'b--', 'LineWidth',1.5);

end