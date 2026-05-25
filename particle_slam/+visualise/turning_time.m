function turning_time(is_turning_hist, no)
figure(no);
grid on;

plot(is_turning_hist, "LineWidth", 1.5);
xlabel('Time Step');
ylabel('turning');
title('Turning Time');
    
end