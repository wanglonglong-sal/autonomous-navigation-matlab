function env = make_env()
    % Figure max size
    env.x_min = -5;
    env.y_min = -5;
    env.x_max = 1000;
    env.y_max = 1000;

    % Obstacle position
    env.poly = {[300, 500; 200, 200; 450, 300], [550,450; 530,300; 700,250; 750, 350], ...
            [600,700; 650,500; 750,600; 800,780], [310,750; 340,600; 500,550; 480,660]};

    % Initial position
    env.start = [0, 0];

    % Goal position
    env.goal = [600, 600];
    % env.goal = [200, 200];

    % Goal reaching tollerance
    env.goal_radius = 50.0;

end