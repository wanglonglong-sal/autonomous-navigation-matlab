function [is_turn] = min_distance_corners(car, env)

% 变量初始化
is_turn = 0;

% 检查地图中弯的数量
n = size(env.corners, 2);

for i = 1:n
    % 获取转弯坐标
    p = env.corners{i};
    cn_x = p(1);
    cn_y = p(2);

    % 计算车辆与转弯点之间距离
    d = norm([car.x - cn_x, car.y - cn_y]);

    % 当距离在转弯点半径内，触发转弯
    if d < env.corner_radius
        is_turn = 1;
        break;
    end
end

end