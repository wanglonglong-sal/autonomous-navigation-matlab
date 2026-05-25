function short_path = rrt_shortcut_opt(order_final_path, env)

poly = env.poly;
n = size(order_final_path, 1);
idx = 1;
n_s_node = 1;
short_path = order_final_path(n_s_node,:);

while idx ~= n 
    for j = idx:n
        collision = path_planning.rrt.chk_collision([order_final_path(idx,3:4); order_final_path(j,3:4)], poly);
        if j < n
            if collision == 0
                % idx = idx + 1;
            else
                idx = j - 1;
                n_s_node = n_s_node + 1;
                short_path(n_s_node,:) = order_final_path(idx,:);
                break;
            end
        else
            if collision == 0
                n_s_node = n_s_node + 1;
                idx = j;
                short_path(n_s_node,:) = order_final_path(idx,:);
                break;
            else
                idx = j - 1;
                n_s_node = n_s_node + 1;
                short_path(n_s_node,:) = order_final_path(idx,:);
                idx = idx + 1;
                n_s_node = n_s_node + 1;
                short_path(n_s_node,:) = order_final_path(idx,:);                
                break;                
            end
          
        end

    end

end
% draw the final path
plot(short_path(:,3), short_path(:,4), 'k-', 'LineWidth', 2);
end