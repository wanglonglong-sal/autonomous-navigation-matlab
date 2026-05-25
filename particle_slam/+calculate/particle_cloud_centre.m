function [mx, my, mth] = particle_cloud_centre(particle)

xs = [particle.x];
ys = [particle.y];
th = [particle.th];
w = [particle.w];
mx = sum(xs .* w);
my = sum(ys .* w);
mth = atan2(sum(sin(th) .* w), sum(cos(th) .* w));

end