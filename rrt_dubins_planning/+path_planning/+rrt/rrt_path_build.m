function [order_final_path] = rrt_path_build(n_added, P_list, goal)
    final_path = nan(n_added, 4);
    % build the final path
    n_real_node = size(P_list(~isnan(P_list(:,1)),:,1),1);
    clue  = n_added;

    for i = 1:n_real_node

        % Set the final one to path list
        final_path(i,:) = P_list(clue,:);
        % Set new clue
        clue = final_path(i,1:1);

        if clue == 0
            break;
        end

    end

    % inverse the order of final path as 1 - N
    order_final_path = flipud(final_path);
    % delete nan clomns
    order_final_path = order_final_path(~all(isnan(order_final_path),2),:);
    % add the goal in the end of the path
    n_final_path = size(order_final_path,1);
    order_final_path(n_final_path+1,:) = [n_final_path, n_final_path + 1, goal];

    % draw the final path
    plot(order_final_path(:,3), order_final_path(:,4), 'b-', 'LineWidth', 2);
end