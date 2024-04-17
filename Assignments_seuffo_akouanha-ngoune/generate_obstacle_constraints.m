function  kx = generate_obstacle_constraints(obstacles, X1, Y1, min_distance, original_kx)
    % Initialize constraints
    max_e1 = 3.7;
    min_e1 = 3;
    max_e2 = 3*pi;
    kx = original_kx;
    
    % Check for each obstacle
    for i = 1:size(obstacles, 1)
        obstacle = obstacles{i};
        % Check if vehicle is close to obstacle
        if is_close_to_obstacle(obstacle(1), obstacle(2), X1, Y1, min_distance) 
           kx = [ 
               min_e1;
               max_e1
               max_e2;
               max_e2;
               ];
        else 
            kx = original_kx; 
        end
    end
end
