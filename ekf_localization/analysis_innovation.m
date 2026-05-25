function analysis_innovation(r_inno_hist, b_inno_hist, t)

    % Draw a range innovation cruve
    figure; grid on; hold on;
    plot(t(1:end-1), r_inno_hist);
    title('Range Innovation');
    xlabel('t [s]');
    ylabel('range innovation [m]');

    % Print Mean of range innovation
    % - E[z~k​]≈0 - unbiasedness
    all_r = r_inno_hist(:);
    all_r = all_r(~isnan(all_r));
    mu_r = mean(all_r);
    fprintf('Mean range innovation: %.4f m\n', mu_r);

    % Print Standard deviation of range innovation
    % - Variance Matching - compare the std_r with the measurement noise (R) range setting
    std_r = std(all_r);
    fprintf('Standard Deviation of range innovation: %.4f m\n', std_r);    

    % Draw a bearing innovation cruve
    figure; grid on; hold on;
    plot(t(1:end-1), b_inno_hist);
    title('Bearing Innovation');
    xlabel('t [s]');
    ylabel('Bearing innovation [rad]');

    % Calculate and print mean of bearing innovation
    % - E[z~k​]≈0 - unbiasedness
    all_b = b_inno_hist(:);
    all_b = all_b(~isnan(all_b));
    mu_b = mean(all_b);
    fprintf('Mean bearing innovation: %.4f rad\n', mu_b);

    % Print Standard deviation of range innovation
    % - Variance Matching - compare the std_b with the measurement noise (R) bearing setting
    std_b = std(all_b);
    fprintf('Standard Deviation of bearing innovation: %.4f rad\n', std_b);    

end
