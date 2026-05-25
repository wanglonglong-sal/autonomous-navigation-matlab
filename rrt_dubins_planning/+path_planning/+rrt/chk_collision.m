
function result = chk_collision(line, poly)
% plotting
% chk_collision_plot(line, poly);

result = 0;

% poly should be a cell array, cound how many polygon we need to check
shape_count = size(poly, 2);

for i = 1:shape_count
    crnt_poly = poly{i};
    poly_size = size(crnt_poly);  % the ith perticular segments
    
    % polygon should consists of at least 3 points
    if poly_size(1) <= 2
        result = -2;
        return;                 % error, incorrect polymer definition
    end

    % check line segement return to initial point.
    % If not, add initial point as ending point to encircle the polygon
    if any(crnt_poly(poly_size(1), :) ~= crnt_poly(1, :))
        poly_size(1) = poly_size(1)+1;
        crnt_poly(poly_size(1), :) = crnt_poly(1, :);    
    end
    seg_count = poly_size(1) - 1;   % segment is one count lesser then vertex
    
    dA = line(2,:) - line(1,:);                                     % dimension should be [1, 2]    
    dB = crnt_poly(2:poly_size(1), :) - crnt_poly(1:poly_size(1)-1, :);       % dimension should be [seg_count, 2]
    dA1B1 = repmat(line(1,:), [seg_count, 1]) - crnt_poly(1:seg_count, :);% dimension should be [seg_count, 2]
    denominator = dB(:, 2) .* dA(1) - dB(:, 1) .*dA(2);
    if all(denominator == 0)         % all lines are parrel, which is very unlikely 
        result = 0;
        return;
    end
    ua = dB(:, 1) .* dA1B1(:, 2) - dB(:, 2) .* dA1B1(:, 1);
    ub = dA1B1(:, 2) .* dA(1) - dA1B1(:, 1) .* dA(2);
    ua = ua ./ denominator;
    ub = ub ./ denominator;
    if ( all( ((ua<0)|(ua>1))|((0>ub)|(ub>1)) ) )
        result = 0;
    else
        result = 1;
        return;
    end
end

end
