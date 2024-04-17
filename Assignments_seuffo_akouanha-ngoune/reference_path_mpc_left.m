function [x, y, psidot_des_mpc, psi_des_mpc, s] = reference_path_mpc_left(Vx, R, L_straight, N_points)
    % Define total path length (straight + circular arc)
    s_total = L_straight + pi * (R - 1.75); % Decreased radius by 1.75m for new curve
    s = linspace(0, s_total, N_points); % Discretize path
    
    % Pre-allocate arrays
    x = zeros(1, N_points);
    y = zeros(1, N_points);
    psidot_des_mpc = zeros(1, N_points); % Rate of change of heading
    psi_des_mpc = zeros(1, N_points); % Desired heading
    
    % Loop through each point to set values
    for i = 1:N_points
        if s(i) <= L_straight
            % Straight segment
            x(i) = s(i) - 1.75 ;
            y(i) = 1.75; % Move up the straight path by 1.75m
            psidot_des_mpc(i) = 0; % No heading change on straight
        else
            % Circular segment
            theta = (s(i) - L_straight) / (R - 1.75); % Decreased radius by 1.75m for new curve
            x(i) = L_straight + (R - 1.75) * sin(theta);
            y(i) = (R - 1.75) * (1 - cos(theta)) + 1.75; % Positive sign to place it above
            psidot_des_mpc(i) = Vx / (R - 1.75); % Constant rate of heading change in the circle
        end
        
        % Calculate heading angle at each point
        psi_des_mpc(i) = atan2(y(i), x(i)); 
    end
end
