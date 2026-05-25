function fig2 = N_eff_particle(N_eff_hist, N_th, no)
fig2 = figure(no);
grid on;

plot(N_eff_hist, "LineWidth", 1.5);
xlabel('Time Step');
ylabel('N_{eff}');
title('Effective Particle Number');
yline(N_th,'r--','Threshold')
end