function fig = particle_error(hist, no)
fig = figure(no);
grid on;
hold on;

% Error in x
plot(hist(1,:), 'r', "LineWidth", 1.5);
% Error in y
plot(hist(2,:), 'g', "LineWidth", 1.5);
% Error in theta
plot(hist(3,:), 'b', "LineWidth", 1.5);

xlabel('Time Step');
ylabel('Error');
title('Particle Centre Error');
legend('x error','y error','theta error');

end