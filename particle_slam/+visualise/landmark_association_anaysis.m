function landmark_association_anaysis(t_n_match, t_n_amb, t_n_new, no)

fig = figure(no);
grid on;
hold on;

% Error in x
plot(t_n_match, 'r', "LineWidth", 1.5);
% Error in y
plot(t_n_amb, 'g', "LineWidth", 1.5);
% Error in theta
plot(t_n_new, 'b', "LineWidth", 1.5);

xlabel('Time Step');
ylabel('Count');
title('Landmark Association Action');
legend('Match','Ambiguity','New');

end
