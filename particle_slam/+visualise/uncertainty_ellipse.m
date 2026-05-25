function uncertainty_ellipse(particle, hMean, hEll)

% Calculate the center of the ellipse
[mx, my, mth] = calculate.particle_cloud_centre(particle);
xs = [particle.x];
ys = [particle.y];
set(hMean,'XData',mx,'YData',my);

% Calculate the Covariance Matrix of particle
C = cov(xs, ys); 
C = (C + C')/2;    

% V: eigenvector; D: eigenvalues
[V,D] = eig(C);
% Gain lamuda1(semi-long-axis) and lamuda2(semi-short-axis)
d = diag(D);
d(d < 0) = 0;                     

% Zoo out 
k = 10;

% generate angular from 0 to 6.28                            
t = linspace(0,2*pi,60);
% generate a circle
circle = [cos(t); sin(t)];

% ellipse = V * √D * circle
A = V * diag(sqrt(d));            
E = [mx; my] + k * (A * circle);

set(hEll,'XData',E(1,:),'YData',E(2,:));    

end