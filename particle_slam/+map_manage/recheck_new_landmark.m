function is_exist_lm = recheck_new_landmark(mu, particle, e)

n = length(particle.landmark);
is_exist_lm = 0;

for i = 1:n

    dx = mu(1) -  particle.landmark(i).mu(1);
    dy = mu(2) -  particle.landmark(i).mu(2);
    d = norm([dx, dy]);

    if d < e
        is_exist_lm = 1;
        break;
    end

end

end