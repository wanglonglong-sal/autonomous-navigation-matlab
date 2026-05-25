function analysis_NIS(NIS_hist, m, c_range)

    % Flatten nis data
    all_nis = NIS_hist(:);
    all_nis = all_nis(~isnan(all_nis));

    figure; hold on; grid on;

    % draw all data as .
    plot(all_nis, '.');
    % draw the range limitation 
    upper = chi2inv(c_range, m);
    lower = chi2inv((1-c_range), m);
    yline(upper, 'r--');
    yline(lower, 'r--');
    
    title('NIS Consistency Check');
    xlabel('measurement index');
    ylabel('NIS');

end 