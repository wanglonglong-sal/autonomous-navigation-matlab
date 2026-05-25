function start_end_plot(env)

% Draw the start point
plot(env.start(1), env.start(2), 'ro',"LineWidth", 2, "MarkerSize", 8, "MarkerFaceColor", 'r');

% Draw the goal point
plot(env.goal(1), env.goal(2), 'ko',"LineWidth", 2, "MarkerSize", 8, "MarkerFaceColor", 'k');

end