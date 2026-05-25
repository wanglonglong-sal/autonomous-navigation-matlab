function [x, y] = car_perception_area(car, car_x, car_y)
r = car.sensor_r;
t = linspace(0, 2*pi, 100);

x = car_x + r*cos(t);
y = car_y + r*sin(t);
  
end