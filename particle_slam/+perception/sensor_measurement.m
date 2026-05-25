function candi_landmarks = sensor_measurement(env, car, std_range, std_bearing)

cl_idx = 1;
candi_landmarks = nan(size(env.landmarks,2), 4);

car_x = car.state(1);       % [m]
car_y = car.state(2);       % [m]
car_th = car.state(3);     % [rad]

n_L = size(env.landmarks,2);

for k = 1:n_L
    d_car_L = inf;
    landmark = env.landmarks{k};
    l_x = landmark(1);
    l_y = landmark(2);
    dt_x = l_x - car_x;
    dt_y = l_y - car_y;
    d_car_L = norm([dt_x, dt_y]) + randn * std_range;
    d_car_L = max(d_car_L, 0);
    th_car_L = atan2(dt_y, dt_x) - car_th + randn * std_bearing;
    th_car_L = wrapToPi(th_car_L);
    if d_car_L < car.sensor_r
        candi_landmarks(cl_idx,:) = [env.landmarks{k}, d_car_L, th_car_L];
        cl_idx = cl_idx + 1;
    end
end

end