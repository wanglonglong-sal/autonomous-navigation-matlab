function animate_ekf(x_hist, P_hist, l, t)
    %% ==============================================================
    %  CANVAS INITIALISATION
    % ===============================================================

    % Open a empty canvas for drawing
    figure; clf;

    % Show grid
    grid on;

    % mulity layers 
    hold on;

    % x-axis equals y-axis
    axis equal;

    % add label for x and y-axis
    xlabel('x [m]');
    ylabel('y [m]');

    N_u = length(t) - 1;

    %% ==============================================================
    %  ANIMATION PROCESS
    % ===============================================================

    % Draw all landmarks
    plot(l(:,1), l(:,2), 'kx', 'LineWidth', 1.5, 'MarkerSize', 8);

    % Initialized a trajectory object
    traj = plot(nan, nan, '-', 'LineWidth', 1.5);

    % Initialized a car object
    car = plot(nan, nan, 'bo', 'MarkerSize', 8, 'LineWidth', 2);

    % Initialized a heading object
    head = quiver(0, 0, 0, 0, 0, 'b', 'LineWidth', 2);

    ell = plot(nan, nan, 'r-', 'LineWidth', 1.2);   % ellipse handle

    % Set a heading length
    Lh = 0.3;

    % Animation update process
    for k = 1:N_u

        % Get specific state in each timestep
        xk = x_hist(1,k);
        yk = x_hist(2,k);
        th = x_hist(3,k);

        % update trajectory
        set(traj, 'XData', x_hist(1,1:k), 'YData', x_hist(2,1:k));
        % update car position
        set(car, 'XData', xk, 'YData', yk);
        % update car heading
        set(head, 'XData', xk, 'YData', yk, 'UData', Lh*cos(th), 'VData', Lh*sin(th));

        mu = x_hist(1:2,k);
        Pxy = P_hist(1:2,1:2,k);
        vis_scale = 100; 
        XY = cov_ellipse_points(mu, Pxy, 2*vis_scale);
        set(ell, 'XData', XY(1,:), 'YData', XY(2,:));

        drawnow;
        pause(0.01);
        
    end
end

function XY = cov_ellipse_points(mu_xy, P_xy, n_sigma)

    if nargin < 3 || isempty(n_sigma), n_sigma = 2; end

    P_xy = 0.5 * (P_xy + P_xy');
    [V, D] = eig(P_xy);
    [d, idx] = sort(diag(D), 'descend');
    V = V(:, idx);
    d = max(d, 0);

    a = n_sigma * sqrt(d(1));
    b = n_sigma * sqrt(d(2));

    tt = linspace(0, 2*pi, 200);
    E = [a*cos(tt); b*sin(tt)];
    XY = V * E + mu_xy(:);
end
