function analysis_P(P_hist, t)
    
    % Get variance x and sqrt
    sig_x = sqrt(squeeze(P_hist(1,1,:)));
    % Get variance y and sqrt
    sig_y = sqrt(squeeze(P_hist(2,2,:)));
    % Get variance th and sqrt
    sig_th = sqrt(squeeze(P_hist(3,3,:)));

    figure; grid on; hold on;

    plot(t, sig_x, 'LineWidth', 1.5);
    plot(t, sig_y, 'LineWidth', 1.5);
    plot(t, sig_th, 'LineWidth', 1.5);

    xlabel('t[s]');
    ylabel('Standard Deviation (1-\sigma)');
    legend('\sigma_x [m]',  '\sigma_y [m]', '\sigma_\theta [rad]');
    title('State Uncertainty Evolution');

end