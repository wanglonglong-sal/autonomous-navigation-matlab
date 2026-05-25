function init_map(env, no)

figure(no); hold on; grid on;
axis([env.x_min env.x_max env.y_min env.y_max]);

% Draw the start and end point
common.start_end_plot(env);

% Draw obstacles
common.obstacles_poly_plot(env.poly);   

end