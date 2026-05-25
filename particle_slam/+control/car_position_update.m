function car = car_position_update(car, current_v, dt)
    car.x = car.x + current_v * cos(car.th) * dt;
    car.y = car.y + current_v * sin(car.th) * dt;
    car.state = [car.x; car.y; car.th];
end