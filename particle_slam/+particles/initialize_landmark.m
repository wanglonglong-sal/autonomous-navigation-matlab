function [mu, Sigma] = initialize_landmark(particle, r_t, b_t, R)

mu_x = particle.x + r_t * cos(b_t + particle.th);
mu_y = particle.y + r_t * sin(b_t + particle.th);
mu = [mu_x; mu_y];

% Initial uncertainty
G = [cos(b_t + particle.th), r_t * -sin(b_t + particle.th);
        sin(b_t + particle.th), r_t * cos(b_t + particle.th)
];

Sigma = G * R * G'; 

end