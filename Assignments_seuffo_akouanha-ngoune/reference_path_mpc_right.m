function [x, y, psidot_des_mpc, psi_des_mpc, s] = reference_path_mpc_right(Vx, R, L_straight, N_points, obstacle_1x, obstacle_1y, obstacle_2x, obstacle_2y)
    % Parameters
    R = R + 1.75; % Increased radius for the circular part
    shift = 3.5; % Shift amount for obstacle avoidance
    s_total = L_straight + pi * R; % Total path length
    s = linspace(0, s_total, N_points); % Discretize path

    % Pre-allocate arrays
    x = zeros(1, N_points);
    y = zeros(1, N_points);
    psi_des_mpc = zeros(1, N_points); % Desired heading
    psidot_des_mpc = zeros(1, N_points); % Rate of change of heading

    % Generate path coordinates
    for i = 1:N_points
        if s(i) <= L_straight
            % Straight segment
            x(i) = s(i) + 1.75;
            y(i) = -1.75;
        else
            % Circular segment
            theta = (s(i) - L_straight) / R;
            x(i) = L_straight + R * sin(theta);
            y(i) = R * (1 - cos(theta)) - 1.75;
        end
    end

    % Calculate initial heading angles
    for i = 2:N_points
        psi_des_mpc(i) = atan2(y(i) - y(i-1), x(i) - x(i-1));
    end
    psi_des_mpc(1) = psi_des_mpc(2); % Initialize the first heading value

    % Adjust path for obstacles
    for i = 1:N_points
        d_obstacle1 = sqrt((x(i) - obstacle_1x)^2 + (y(i) - obstacle_1y)^2);
        d_obstacle2 = sqrt((x(i) - obstacle_2x)^2 + (y(i) - obstacle_2y)^2);
        
        % Determine shifts based on proximity to both obstacles
        if d_obstacle2 < 20
            % Shift left as we approach the obstacle
            x(i) = x(i) - shift ;
        elseif d_obstacle2 < 40 && d_obstacle2 >= 20
            
            % Gradually move back to the original path after passing the obstacle
            x(i) = x(i) - shift * (40 - d_obstacle2) / 20;
        end
         
        if d_obstacle1 < 20
            % Shift upward as we approach the obstacle
            y(i) = y(i) + shift;
        end
    end

    % Recalculate heading after adjustments
    for i = 2:N_points
        psi_des_mpc(i) = atan2(y(i) - y(i-1), x(i) - x(i-1));
    end
    psi_des_mpc(1) = psi_des_mpc(2); % Initialize the first heading 

    % Compute the rate of change of heading angle
    for i = 2:N_points
        psidot_des_mpc(i) = (psi_des_mpc(i) - psi_des_mpc(i-1)) / (s(i) - s(i-1)) * Vx;
    end
    psidot_des_mpc(1) = psidot_des_mpc(2); % Initialize the first rate of change value

end
