function car = make_car(start)

% Car initial position
car.x = start(:,1);     %[m]
car.y = start(:,2);     %[m]

% Car initial heading
car.th = deg2rad(45);   %[rad]

% Car Initial State
car.state = [car.x, car.y, car.th];

% Car minimum return radius
car.r = 5;              %[m]

% Linear velocity
car.v = 5;                  %[m/s]

% Maximum angular velocity
car.w_max = 1.0;            %[rad/s]

% Maximum lateral acceleration 
car.u_max = 5.0;            %[m/s^2]

end