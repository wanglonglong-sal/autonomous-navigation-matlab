function particle = landmark_washing(particle, gate_new)

n_particle = length(particle);

for i = 1:n_particle

    n_landmark = length(particle(i).landmark);
    if n_landmark > 1

        for j = 1:n_landmark

            for k = j+1:n_landmark

                if ~isnan(particle(i).landmark(j).mu(1)) && ~isnan(particle(i).landmark(k).mu(1)) && ...
                   ~isnan(particle(i).landmark(j).mu(2)) && ~isnan(particle(i).landmark(k).mu(2)) 
                    dx = particle(i).landmark(j).mu(1) - particle(i).landmark(k).mu(1);
                    
                    dy = particle(i).landmark(j).mu(2) - particle(i).landmark(k).mu(2);
        
                    d = norm([dx, dy]);

                    if d < gate_new
                        if particle(i).landmark(j).update > particle(i).landmark(k).update
                            particle(i).landmark(k).mu = nan;
                            particle(i).landmark(k).Sigma = nan;
                            particle(i).landmark(k).update = nan;
                        else
                            particle(i).landmark(j).mu = nan;
                            particle(i).landmark(j).Sigma = nan;
                            particle(i).landmark(j).update = nan;                            
                        end
                    end
                end
            end

        end
    end

    idx = ~arrayfun(@(l) isnan(l.mu(1)), particle(i).landmark);
    particle(i).landmark = particle(i).landmark(idx);

end

end