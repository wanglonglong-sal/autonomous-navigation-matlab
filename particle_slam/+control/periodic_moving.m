function  [car, w_v] = periodic_moving(car, env, dt)
    % reference sinusoidal path
    A_path = 40;          % 振幅，先试 30~50
    k_path = 0.03;        % 空间频率，越大波越密
    x0 = env.start(1);    % 起点 x
    y0 = env.start(2);    % 中心线 y
    w_v = 0;

    % desired curve: y = y0 + A*sin(k*(x-x0))
    dy_dx = A_path * k_path * cos(k_path * (car.x - x0));
    th_ref = atan2(dy_dx, 1);

    % heading control
    k_th = 1.5;
    w_v = k_th * wrapToPi(th_ref - car.th);
    w_v = max(min(w_v, car.w_max), -car.w_max);

    % update heading
    car.th = wrapToPi(car.th + w_v * dt);
end