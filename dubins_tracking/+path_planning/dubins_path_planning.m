function D_list = dubins_path_planning(waypoint_s, waypoint_f, cc_list_s, cc_list_f, r_radius)
    % Initialised candidate list
    candi_list = nan(4, 9);
    % LSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('L','L',waypoint_s, waypoint_f, cc_list_s, cc_list_f, r_radius);
    candi_list(1,:) = [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];
    % RSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('R','R',waypoint_s, waypoint_f, cc_list_s, cc_list_f, r_radius);
    candi_list(2,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];    
    % LSR
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('L','R',waypoint_s, waypoint_f, cc_list_s, cc_list_f, r_radius);
    candi_list(3,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];  
    % RSL
     [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f] = path_planning.CSC('R','L',waypoint_s, waypoint_f, cc_list_s, cc_list_f, r_radius);
    candi_list(4,:) =  [d, t_s, the_0_s_t, t_f, the_0_f_t, the_0_s_s, the_0_f_f];   

    % Gain the minimun value
    vals = candi_list(:,1);
    vals(~isfinite(vals)) = nan;
    [min_val, idx] = min(vals, [], "omitnan");

    % Sort of Dataset
    switch(idx)
        case 1
            C1 = 76; %'L'
            C2 = 76; %'L'
            cc_s = cc_list_s(1:2);
            cc_f = cc_list_f(1:2);
        case 2
            C1 = 82; %'R'
            C2 = 82; %'R'
            cc_s = cc_list_s(3:4);
            cc_f = cc_list_f(3:4);
        case 3
            C1 = 76; %'L'
            C2 = 82; %'R'
            cc_s = cc_list_s(1:2);
            cc_f = cc_list_f(3:4);
        case 4
            C1 = 82; %'R'
            C2 = 76; %'L'
            cc_s = cc_list_s(3:4);
            cc_f = cc_list_f(1:2); 
    end

    D_list = [idx, waypoint_s, waypoint_f, cc_s, cc_f, candi_list(idx,:), C1, C2];
end
