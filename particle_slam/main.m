clc; clear; close all;

% rng(20260308, "twister");   

%% ==============================================================
%  Environment Initialisation
% ===============================================================

figure_no = 1;

% Initialise ENV
env = common.make_env();

% Canvas Initialisation
fig1 = common.init_map(env, figure_no);
figure_no = figure_no + 1;

% Initialise Car
car = common.make_car(env.start);

% Time Settings
dt = 0.2;   %[s]
T = 500;   %[s]
N_t = T/dt;   %[times]

%% ==============================================================
%  Control Initilisation
% ===============================================================

% ----- Car State -----
% car_x = car.x;       % [m]
% car_y = car.y;       % [m]
% car_th = car.th;     % [rad]
car_x_hist = nan(1, N_t);
car_y_hist = nan(1, N_t);

% ----- Turning Settings -----
e_th = 0.0;    % expect theta
is_turn = 0;   % turning or not
MAX_COOL_SPACE = 20; % Maximum Cooldown space
cooldown = MAX_COOL_SPACE; % turning protect step

%% ==============================================================
%  Particle Algorithm Initialisation
% ===============================================================
% ----- Particle Parameters -----
n_particles = 200;

% ----- Particle State Error Standard Deviation -----
std_x0 = 0.2;             %[m]
std_y0 = 0.2;             %[m]
std_theta0 = deg2rad(2);  %[rad]

% ----- Process Noise  -----
std_lin_vel = 0.007;     % [m/s]
std_ang_vel = 0.0075;    % [rad/s]

% ----- Measurement Noise and Covariance Matrix (R) -----
% std_range = 0.02;        %[m]
std_range = 2;        %[m]
% std_bearing = 0.15;      %[rad]
std_bearing = deg2rad(5);      %[rad]
R = diag([std_range^2, std_bearing^2]);

% ----- Landmark association coefficiency  -----
gate_update = 9.21;
gate_new = 50;
gate_wash = 15;
gate_recheck = 3;
p_amb = 0.7;
p_new = 0.8;


% ----- Resample standard deviation  -----
std_resample_x = 0.002;              % m
std_resample_y = 0.002;              % m
std_resample_th = deg2rad(0.1);      % rad

% ----- Effective Particle Number Thredshold -----
N_th = 0.3 * n_particles;
N_eff_hist = zeros(1, N_t);
MAX_RESAMPLE_COOL_SPACE = 20;
resample_cooldown = 0;
did_resample = false;

% ----- Error Analysis -----
particle_err_hist = zeros(3, N_t);

% ----- Paricle animation  -----
hMean = plot(nan,nan,'bo','MarkerSize',6,'LineWidth',2); 
hEll  = plot(nan,nan,'b-','LineWidth',1.5); 
hParticle = plot(nan, nan, 'r.');
best_map_landmark = plot(nan, nan, 'rx', 'LineWidth',1.5);

%% ==============================================================
%  Particle Initialisation 
% ===============================================================
% 0. Initialization particles
for j = 1:n_particles
    particle(j).x =  car.x + randn * std_x0;
    particle(j).y =  car.y + randn * std_y0;
    particle(j).th =  car.th + randn * std_theta0;
    particle(j).w = 1/n_particles;
    particle(j).landmark = [];
    % plot(particle(j).x, particle(j).y, 'rx', "MarkerSize", 1, "LineWidth", 1);
end

%% ==============================================================
%  Main Process
% ===============================================================
for i = 1:N_t

    % 0.局部变量初始化
    n_match = 0;  % 计数器 - landmark匹配
    n_amb = 0;    % 计数器 - landmark模糊（匹配失败未达到新建）
    n_new = 0;    % 计数器 - landmark新建
    did_resample = false; % 开关 - 是否重采样

    % 0.1 保存车辆运动历史轨迹 - 画轨迹用
    car_x_hist(i) = car.x;
    car_y_hist(i) = car.y;

    % 1.0 车辆行驶轨迹控制 - 路线1 - 车辆环形左转
    [car, w_v] = control.rectangle_moving(car, env, dt, MAX_COOL_SPACE);
    if car.turning == true
        is_turning_hist(i) = 1;
    else
        is_turning_hist(i) = 0;
    end
    % 1.0.1 路线1 - 转弯过程保护
    if car.turning_cool > 0
        car.turning_cool = car.turning_cool - 1;
    end    

    % 1.1 路线2 - 车辆绕x轴周期运动
    % [car, w_v] = control.periodic_moving(car, env, dt);

    % 1.2 车辆当前速度计算 - 转弯降速计算
    current_v = min(car.v, sqrt(car.u_max/(abs(w_v)+eps)));

    % 1.3 车辆位置计算并更新
    car = control.car_position_update(car, current_v, dt);

    % 2.0 粒子状态预测 Particle State Prediction
    particle = particles.state_prediction(particle, n_particles, current_v, w_v, dt, std_lin_vel, std_ang_vel); 
   
    % 3.0 雷达传感器观测返回地标信息（r,b）
    candi_landmarks = perception.sensor_measurement(env, car, std_range, std_bearing);
    candi_landmarks = candi_landmarks(~isnan(candi_landmarks(:,1)),:);
    n_c_l = size(candi_landmarks, 1);

    % 3.1 轮速计信号返回（s）
    % To Do

    % 4.0 地标测量预测及地标状态更新
    if n_c_l > 0    % 本次雷达观测到地标
        % 4.0.1 对particle中map进行处理
        for j = 1:n_particles
            for k = 1:n_c_l
                % 检查该particle中保存的地标数量
                n_p_l = size(particle(j).landmark,2);
                % 提取测量到该地标时传感器返回的距离及航向角真值信息
                r_t = candi_landmarks(k, 3);
                b_t = candi_landmarks(k, 4);
                % Particle内地标索引管理 - 原始新建，更新，尾部新建
                lm_id = 1;   
                
                % 当Particle中地标信息为空，进行原始新建
                if n_p_l == 0
                    % 计算地标相对位置与地标状态不确定性
                    [mu, Sigma] = particles.initialize_landmark(particle(j), r_t, b_t, R);
                    particle(j).landmark(lm_id).mu = mu;
                    particle(j).landmark(lm_id).Sigma = Sigma;
                    particle(j).landmark(lm_id).update = 1;
                    n_new = n_new + 1;
                % 当Particle中地标信息不为空，进行匹配比对
                else
                    % 初始化粒子地图匹配列表
                    candi_pl_list = struct('idx',{},'d2',{},'H',{},'S',{}, 'innov',{});

                    % Find the same landmark in particles
                    for p = 1:n_p_l
                        mu = particle(j).landmark(p).mu;
                        Sigma = particle(j).landmark(p).Sigma;
                        dx = mu(1) - particle(j).x;
                        dy = mu(2) - particle(j).y;
                        r_p = norm([dx, dy]);
                        r_p = max(r_p, 1e-3);
                        b_p = wrapToPi(atan2(dy, dx) - particle(j).th);

                        innov = [r_t - r_p; wrapToPi(b_t - b_p)];                        
                        H = [dx/r_p,      dy/r_p;
                            -dy/r_p^2,    dx/r_p^2];
                        S = H * Sigma * H' + R;
                        S = 0.5*(S+S') + 1e-9*eye(2);

                        d2 = innov' * (S \ innov);

                        candi_pl_list(p).idx = p;
                        candi_pl_list(p).d2 = d2;
                        candi_pl_list(p).H = H;
                        candi_pl_list(p).S = S;
                        candi_pl_list(p).innov = innov;

                    end

                    [min_d2, idx] = min([candi_pl_list.d2]);
                    lm_id = candi_pl_list(idx).idx;
                    % Landmark Association - update case
                    if min_d2 < gate_update
                        H = candi_pl_list(idx).H;
                        S = candi_pl_list(idx).S;
                        innov = candi_pl_list(idx).innov;
                        Sigma = particle(j).landmark(lm_id).Sigma;
                        mu = particle(j).landmark(lm_id).mu;
                        
                        % EKF Update
                        K = Sigma * H' / S;
                        mu = mu + K * innov;
                        % Sigma = (eye(2) - K * H) * Sigma;
                        I2 = eye(2);
                        Sigma = (I2 - K*H) * Sigma * (I2 - K*H)' + K * R * K';
                        Sigma = 0.5 * (Sigma + Sigma');
                        Sigma = Sigma + 1e-5 * eye(2);

                        particle(j).landmark(lm_id).mu = mu;
                        particle(j).landmark(lm_id).Sigma = Sigma;
                        update_count = particle(j).landmark(lm_id).update + 1;
                        particle(j).landmark(lm_id).update = update_count;
                        % d2 = candi_pl_list(idx).d2;
                        % likelihood = exp(-0.5 * min_d2);
                        % likelihood = max(likelihood, 1e-12);

                        S = candi_pl_list(idx).S;
                        detS = det(S);
                        detS = max(detS, 1e-12);          % 防止数值问题
                        norm_const = 1 / (2*pi*sqrt(detS));
                        likelihood = norm_const * exp(-0.5 * min_d2);
                        likelihood = max(likelihood, 1e-12);
        

                        n_match = n_match + 1;

                    % Landmark Association - uncertain case
                    elseif gate_update <= min_d2  && min_d2 < gate_new
                        likelihood = p_amb; % Strong punishment for weight

                        n_amb = n_amb + 1;

                    elseif min_d2 >= gate_new
                    % Landmark Association - create case
                        likelihood = p_new; % Slight punishment for weight

                        [mu, Sigma] = particles.initialize_landmark(particle(j), r_t, b_t, R);
                        
                        is_exist_lm = map_manage.recheck_new_landmark(mu, particle(j), gate_recheck);
                        
                        if is_exist_lm == 0
                            lm_id = length(particle(j).landmark) + 1;                        
                            particle(j).landmark(lm_id).mu = mu;
                            particle(j).landmark(lm_id).Sigma = Sigma;
                            particle(j).landmark(lm_id).update = 1;
                        end
                        
                        n_new = n_new + 1;

                    end
                    particle(j).w = particle(j).w * likelihood;                         

                end
            end
        end
        % Find the max weight and plot its map
        [val, idx] = max([particle.w]);
        n_max_p = length(particle(idx).landmark);
        % plot(particle(idx).landmark(k).mu(1), particle(idx).landmark(k).mu(2), 'rx', 'LineWidth',1.5);
        mu = reshape([particle(idx).landmark.mu], 2, []);
        set(best_map_landmark, ...
            'XData', mu(1,:), ...
            'YData', mu(2,:));

        % 4. Normalization particle weight
        w = [particle.w];
        wsum = sum(w);

        if wsum == 0
            for j = 1:n_particles
                particle(j).w = 1/n_particles;
            end            
        else
            for j = 1:n_particles
                particle(j).w = particle(j).w/wsum;
            end                
        end

        % draw the uncertainty ellipse
        [mx, my, mth] = calculate.particle_cloud_centre(particle);
        particle_err_hist(:,i) = [car.x - mx; car.y - my; wrapToPi(car.th - mth)];
        visualise.uncertainty_ellipse(particle, hMean, hEll);
        set(hParticle, 'XData', [particle.x], 'YData', [particle.y]);

        % 5. Resampling
        w = [particle.w];
        c = cumsum(w); % CDF
        N_eff = 1 / sum(w.^2);
        N_eff_hist(i) = N_eff; 
        
        % When the effective particle number less than thredshold, do resampling
        if resample_cooldown == 0 && N_eff < N_th

            newP = particle;
            step = 1/n_particles;
            r = rand*step;
            idx = 1;

            for j = 1:n_particles

                u = r + (j-1)*step;
                while (idx < n_particles) && (u > c(idx))
                    idx = idx + 1;
                end
                newP(j).x = particle(idx).x + randn * std_resample_x;
                newP(j).y = particle(idx).y + randn * std_resample_y;
                newP(j).th = wrapToPi(particle(idx).th + randn * std_resample_th);
                newP(j).w = 1/n_particles;

            end
            particle = newP;
            did_resample = true;
        end
            

    % 本次雷达没有观测到地标
    else
        % When there is not any landmark detected, also calcuate the effective particle
        w = [particle.w];
        N_eff = 1 / sum(w.^2);
        N_eff_hist(i) = N_eff; 
    end

    % draw the animation of car movement
    L = 10;
    set(car.pose, 'XData', car.x, 'YData', car.y);
    set(car.heading, 'XData', car.x, 'YData', car.y, 'UData', L*cos(car.th), 'VData', L*sin(car.th));
    [p_x, p_y] = calculate.car_perception_area(car, car.x, car.y);
    set(car.perception, 'XData', p_x, 'YData', p_y);
    % set(car.traj, 'XData', car_x_hist(1:i), 'YData', car_y_hist(1:i));

    title(sprintf('Time Step = %.2f s, v = %.2f m/s', i*dt, current_v));

    drawnow;

    % Map Washing
    if mod(i, 200) == 0
        particle = map_manage.landmark_washing(particle, gate_wash);
    end

    t_n_match(i) = n_match;
    t_n_amb(i) = n_amb;
    t_n_new(i) = n_new;



    if did_resample
        resample_cooldown = MAX_RESAMPLE_COOL_SPACE;
    elseif resample_cooldown > 0
        resample_cooldown = resample_cooldown - 1;
    end

end

%% ==============================================================
%  Visualisation and Analysis
% ===============================================================
fig2 = visualise.N_eff_particle(N_eff_hist, N_th, figure_no);
figure_no = figure_no + 1;

visualise.particle_error(particle_err_hist, figure_no);
figure_no = figure_no + 1;

visualise.landmark_association_anaysis(t_n_match, t_n_amb, t_n_new, figure_no);
figure_no = figure_no + 1;

visualise.turning_time(is_turning_hist, figure_no);
figure_no = figure_no + 1;

