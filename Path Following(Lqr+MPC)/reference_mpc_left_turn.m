function [x, y, psidot_des_mpc, psi_des_mpc, s] = reference_mpc_left_turn(Vx, L_straight, L_turn, N_points)
    % Define total path length (straight + turn + another straight)
    s_total = 2 * L_straight + L_turn;
    
    % Discretize path
    s_straight1_points = linspace(0, L_straight, round(N_points * L_straight / s_total));
    s_turn_points = linspace(L_straight, L_straight + L_turn, round(N_points * L_turn / s_total));
    s_straight2_points = linspace(L_straight + L_turn, s_total, round(N_points * L_straight / s_total));
    s = [s_straight1_points, s_turn_points(2:end), s_straight2_points(2:end)]; % Avoid repeating points
    
    % Pre-allocate arrays
    x = zeros(1, length(s));
    y = zeros(1, length(s));
    psidot_des_mpc = zeros(1, length(s)); % Rate of change of heading
    psi_des_mpc = zeros(1, length(s)); % Heading angle
    
    % Loop through each point to set values
    for i = 1:length(s)
        if s(i) <= L_straight

            % First straight segment
            x(i) = s(i);
            y(i) = 0;
            psi_des_mpc(i) = 0; % Heading straight along the x-axis
        elseif s(i) > L_straight && s(i) <= L_straight + L_turn

            % Turn segment
            fraction = (s(i) - L_straight) / L_turn;
            x(i) = L_straight; % Stay at the end of the first straight segment
            y(i) = fraction * L_turn; % Move up in y
            psi_des_mpc(i) = pi/2 * fraction; % Linearly increase heading from 0 to pi/2
        else
            
            % Second straight segment
            x(i) = L_straight;
            y(i) = L_turn + (s(i) - (L_straight + L_turn)); % Continue straight from end of turn
            psi_des_mpc(i) = pi/2; % Continue heading along the y-axis
        end
    end
    
    % Calculate the rate of change of heading angle
    for i = 2:length(s)
        psidot_des_mpc(i) = (psi_des_mpc(i) - psi_des_mpc(i-1)) / (s(i) - s(i-1));
    end
    psidot_des_mpc(1) = psidot_des_mpc(2); % Duplicate for the first point
end
