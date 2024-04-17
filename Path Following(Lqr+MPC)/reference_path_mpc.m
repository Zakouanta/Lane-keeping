function [x, y, psidot_des_mpc, psi_des_mpc, s, obstacle_1x, obstacle_1y, obstacle_2x,obstacle_2y] = reference_path_mpc(Vx, R, L_straight, N_points)
    % Define total path length (straight + circular arc)
    s_total = L_straight + pi * R; % Assume half-circle for simplicity
    s = linspace(0, s_total, N_points); % Discretize path
    
    % Pre-allocate arrays
    x = zeros(1, N_points);
    y = zeros(1, N_points);
    psidot_des_mpc = zeros(1, N_points); % Rate of change of heading
    psi_des_mpc = zeros(1, N_points); % Desired heading
    
     % Define obstacle positions
    obstacle_1x = L_straight / 2;
    obstacle_1y = -1.75;
    obstacle_2x = L_straight + R + 1.75;
    obstacle_2y = 1000;         % Adjust this value according to your requirements


    % Loop through each point to set values
    for i = 1:N_points
        if s(i) <= L_straight
            % Straight segment
            x(i) = s(i);
            y(i) = 0;
            psidot_des_mpc(i) = 0; % No heading change on straight
        else
            % Circular segment
            theta = (s(i) - L_straight) / R;
            x(i) = L_straight + R * sin(theta);
            y(i) = R * (1 - cos(theta));
            psidot_des_mpc(i) = Vx / R; % Constant rate of heading change in the circle
        end
        
        % Calculate heading angle at each point
        psi_des_mpc(i) = atan2(y(i), x(i));
    end
end
