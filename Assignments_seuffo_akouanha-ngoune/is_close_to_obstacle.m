function [close, distance_to_obstacle] = is_close_to_obstacle(obstacle_x, obstacle_y, X1, Y1, min_distance)

    % Distance between each point on the path and the obstacle point
    distance_to_obstacle = sqrt((X1 - obstacle_x).^2 + (Y1 - obstacle_y).^2);
    
    % Check if either of the distances is less than min_distance
    if any(distance_to_obstacle )< min_distance 
        close = true; % Vehicle is close to obstacle
    else
        close = false; % Vehicle is not close to obstacle
    end
end
