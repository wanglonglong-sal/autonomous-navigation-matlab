function [car, w_v] = rectangle_moving(car, env, dt, MAX_COOL_SPACE)

    % 是否触发转弯判定 - 当此时直线行驶且不在转弯冷却内
    if car.turning == false && car.turning_cool == 0 
        % 判断车辆位置是否靠近弯点，是否触发转弯
        is_turn = control.min_distance_corners(car, env);
        % 触发转弯
        if is_turn == true
            % + pi/2 - 向左转90°
            car.ep_th = wrapToPi(car.th + pi/2);
            car.turning = true;
            % car.turning_cool = MAX_COOL_SPACE;
        end
    end

    if car.turning == true
        err_th = wrapToPi(car.ep_th - car.th);
        w_cmd  = max(min(err_th, car.w_max), -car.w_max);

        % 如果下一步会跨过目标角，就直接到目标角，避免累计偏差
        if abs(err_th) <= abs(w_cmd) * dt + 1e-6
            car.th = car.ep_th;          % 关键：强制对齐目标角
            car.turning = false;
            car.turning_cool = MAX_COOL_SPACE;
            w_v = 0;
        else
            w_v = w_cmd;
            car.th = wrapToPi(car.th + w_v * dt);
        end
    else
        w_v = 0;
    end

end