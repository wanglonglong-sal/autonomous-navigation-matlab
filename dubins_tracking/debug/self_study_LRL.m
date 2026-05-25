clc; clear; close all;

r = 5;

initial_point = [0; 10; deg2rad(160)];
waypoint1 = [20; 20; deg2rad(10)];

% draw canvas
figure('Position', [100, 100, 1000, 800]); grid on; hold on;
axis equal;

% 初始化画布，将默认起点、终点位置以及车头朝向画出
draw_waypoints(initial_point);
draw_waypoints(waypoint1);

%% ==============================================================
%  计算左转圆位置
% ===============================================================
% 绘制计算左转圆的辅助线
L = 10;
% 穿过起点 并平行与 x轴
draw_line(initial_point(1, 1) - L, initial_point(2, 1), initial_point(1, 1) + L, initial_point(2, 1), 'b--');
% 计算左转圆 - 圆心位置 - 起点
l_c_x_1 = initial_point(1, 1) + r * cos(initial_point(3,1) + pi/2);
% 绘制l_c_x_1，观察从起点x，在x轴上的位置移动
draw_point(l_c_x_1, initial_point(2, 1), 'bo');
l_c_y_1 = initial_point(2, 1) + r * sin(initial_point(3,1) + pi/2);
draw_point(initial_point(1, 1), l_c_y_1, 'bo');
% 画出起点圆 - 圆心
plot(l_c_x_1, l_c_y_1, 'ko', 'MarkerSize', 2, 'LineWidth', 2, 'MarkerFaceColor', 'k');
% 绘制左转圆 - 起点圆
draw_circle(l_c_x_1, l_c_y_1, r, '-b');
% 计算左转圆 - 圆心位置 - 终点
l_c_x_2 = waypoint1(1, 1) + r * cos(waypoint1(3,1) + pi/2);
l_c_y_2 = waypoint1(2, 1) + r * sin(waypoint1(3,1) + pi/2);
% 绘制终点圆 - 圆心
plot(l_c_x_2, l_c_y_2, 'ko', 'MarkerSize', 2, 'LineWidth', 2, 'MarkerFaceColor', 'k');
% 绘制左转圆 - 终点圆
draw_circle(l_c_x_2, l_c_y_2, r, '-b');
% 辅助线 - 从圆心 到 起点
plot([l_c_x_1 initial_point(1, 1)],[l_c_y_1 initial_point(2, 1)], 'k-', 'LineWidth', 1);
% 辅助线 - 起点圆 - x轴
plot([l_c_x_1 - L, l_c_x_1 + L + 20], [l_c_y_1, l_c_y_1], 'k--','LineWidth', 1)
% 辅助线 - 终点圆 - y轴
plot([l_c_x_2, l_c_x_2], [l_c_y_2  - L - 10, l_c_y_2], 'k--','LineWidth', 1)

%% ==============================================================
%  计算圆间切线
% ===============================================================
% 绘制圆心间辅助线
plot([l_c_x_1 l_c_x_2],[l_c_y_1 l_c_y_2], 'k--','LineWidth', 1);
% 计算圆心间距离
dx = l_c_x_2 - l_c_x_1;
dy = l_c_y_2 - l_c_y_1;
d = norm([dx, dy]);
% 计算圆心间夹角
alpha = atan2(dy, dx);
% 计算起点圆切点 - 由于两个同样半径的圆，他们之间圆心的连线平行于外公切线。而公切线又垂直于半径。所以圆心之间的夹角转90°就是x,y移动的方向向量
cc_theta = alpha - pi/2;
tang_x_s = l_c_x_1 + r * cos(cc_theta);
tang_y_s = l_c_y_1 + r * sin(cc_theta);
draw_point(tang_x_s, tang_y_s, 'ro');
% 辅助线 - 半径 起点圆心到切点
draw_line(l_c_x_1, l_c_y_1, tang_x_s, tang_y_s, 'r--');

% 计算终点圆切点
tang_x_f = l_c_x_2 + r * cos(cc_theta);
tang_y_f = l_c_y_2 + r * sin(cc_theta);
% 辅助线 - 半径 终点圆心到切点
draw_point(tang_x_f, tang_y_f, 'ro');
% 辅助线 - 半径 终点圆心到切点
draw_line(l_c_x_2, l_c_y_2, tang_x_f, tang_y_f, 'r--');
% 辅助线 - 两切点之间连线
draw_line(tang_x_s, tang_y_s, tang_x_f, tang_y_f, 'y-')

%% ==============================================================
%  计算起点圆弧长 + 直线 + 终点圆弧长距离
% ===============================================================

% LSL 和 RSR 走外公切线，公切线长度等于圆心之间的距离；但是LSR,RSL走内公切线，距离推论不成立。保险起见，重新计算
S_L = norm([tang_y_f - tang_y_s, tang_x_f - tang_x_s]);

% 计算起点角 - 从0°到起点这段半径偏转角度 - 而非heading（heading是从0°偏转到heading箭头，不是半径偏转角度 - 这个地方其实就是cos(the + pi/2)这个部分
theta_r_s = atan2(initial_point(2,1) - l_c_y_1, initial_point(1,1) - l_c_x_1);
% 计算切点角
theta_r_s_t = atan2(tang_y_s - l_c_y_1, tang_x_s - l_c_x_1);
% 计算转角差 = 切点角 - 起点角
delta_theta_s_t = theta_r_s_t - theta_r_s;
% 计算起点圆弧长
L_arc_s = r * delta_theta_s_t;
% 绘制圆弧
draw_arc_s(l_c_x_1, l_c_y_1, r, theta_r_s, theta_r_s_t, 'y-')

% 计算终点角
theta_r_f = atan2(waypoint1(2, 1) - l_c_y_2, waypoint1(1, 1) - l_c_y_1);
% 计算终点圆 -切点角
theta_r_f_t = atan2(tang_y_f - l_c_y_2, tang_x_f - l_c_x_2);
% 计算转角差 = 切点角 - 终点角
delta_theta_f_t = theta_r_f_t - theta_r_f;
% 计算终点圆弧长
L_arc_f = r * delta_theta_f_t;
% 绘制圆弧
draw_arc_f(l_c_x_2, l_c_y_2, r, theta_r_f_t, theta_r_f, 'y-')

% 计算总路径长度
final_L = L_arc_s + S_L + L_arc_f;
fprintf("The final distance is %.2f \n", final_L);


%% ==============================================================
%  Draw Waypoint Position and Heading
% ===============================================================
function draw_waypoints(waypoint)
    % Define the length of heading arrow
    L = 5;

    x = waypoint(1);
    y = waypoint(2);
    th= waypoint(3);
    % Draw the specific position
    plot(x, y, 'ko', 'MarkerSize', 8, 'LineWidth', 2);
    % Draw the heading
    quiver(x, y, L*cos(th), L*sin(th), 0, 'r', 'LineWidth', 2);

end

%% ===== 画圆函数 =====
function draw_circle(cx, cy, r, style)
    ang = linspace(0, 2*pi, 200);
    x = cx + r*cos(ang);
    y = cy + r*sin(ang);
    plot(x, y, style, 'LineWidth',1.5);
end

%% ==============================================================
%  Draw A  line
% ===============================================================
function draw_line(x1, y1, x2, y2, style)

    plot([x1, x2], [y1, y2], style, "LineWidth", 1.5);

end
%% ==============================================================
%  Draw A Point
% ===============================================================
function draw_point(x, y, style)

    plot(x, y, style, 'MarkerSize', 4, 'LineWidth', 1, 'MarkerFaceColor', 'k');

end

%% ==============================================================
%  Draw A Arc
% ===============================================================
function draw_arc_s(xc, yc, r, theta1, theta2, style)

    delta = wrapTo2Pi(theta2 - theta1);
    fprintf("delta %.2f \n", delta);

    theta = linspace(theta1, theta1 + delta, 100);
    fprintf("theta1 %.2f \n", theta1);
    fprintf("theta2 %.2f \n", theta2);
    

    x = xc + r*cos(theta);
    y = yc + r*sin(theta);

    plot(x, y, style, 'LineWidth', 1.5);

end

%% ==============================================================
%  Draw A Arc
% ===============================================================
function draw_arc_f(xc, yc, r, theta_entry, theta_final, style)

    delta = wrapTo2Pi(theta_entry - theta_final);
    fprintf("delta %.2f \n", delta);

    theta = linspace(theta_entry, theta_entry + delta, 100);
    fprintf("theta1 %.2f \n", theta_entry);
    fprintf("theta2 %.2f \n", theta_final);
    
    x = xc + r*cos(theta);
    y = yc + r*sin(theta);

    plot(x, y, style, 'LineWidth', 1.5);

end


