function fig = init_map(env, no)

fig = figure(no); 
hold on; grid on;
axis([env.x_min env.x_max env.y_min env.y_max]);

% Draw the start and end point
% common.start_end_plot(env);

% Draw corners
n = size(env.corners, 2);

for i = 1:n
    p = env.corners{i};
    x = p(1);
    y = p(2);
    plot(x, y, 'ko', 'MarkerSize',4);
end

% Draw corners
n_L = size(env.landmarks, 2);

for i = 1:n_L
    L = env.landmarks{i};
    x = L(1);
    y = L(2);
    plot(x, y, 'kx', 'MarkerSize',6);
end

% Draw obstacles
% common.obstacles_poly_plot(env.poly);   

end