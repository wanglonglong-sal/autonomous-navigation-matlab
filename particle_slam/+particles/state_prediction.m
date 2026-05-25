function particle = state_prediction(particle, n_particles, current_v, w_v, dt, std_lin_vel, std_ang_vel)
    for j = 1:n_particles
        % Consideration of process noise randomly
        v = current_v + randn * std_lin_vel;
        w = w_v + randn * std_ang_vel;
        particle(j).th = wrapToPi(particle(j).th + dt * w);
        particle(j).x = particle(j).x + dt * cos(particle(j).th) * v;
        particle(j).y = particle(j).y + dt * sin(particle(j).th) * v;
    end           
end