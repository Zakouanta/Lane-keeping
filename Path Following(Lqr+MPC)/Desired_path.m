function [psidot_des, psi_des, x_des, y_des] = Desired_path(Vx, r, t_i)
    % Desired_path computes the desired path for a vehicle
    
    % Define symbolic variables
    syms psi_des_dot_sym(t) psi_des_sym(t) x_des_sym(t) y_des_sym(t); 
    % Define the piecewise function for psi_des_dot
    psi_des_dot_sym(t) = piecewise(t <= 5, 0, t > 5, Vx/r);
    % Integrate psi_des_dot to get psi_des
    psi_des_sym(t) = int(psi_des_dot_sym(t), 0, t);
    % Integrate Vx*cos(psi_des) to get x_des
    x_des_sym(t) = int(Vx * cos(psi_des_sym(t)), 0, t);
    % Integrate Vx*sin(psi_des) to get y_des
    y_des_sym(t) = int(Vx * sin(psi_des_sym(t)), 0, t);
    % Evaluate the symbolic functions at the given time points
    psidot_des = double(psi_des_dot_sym(t_i));
    psi_des = double(psi_des_sym(t_i));
    x_des = double(x_des_sym(t_i));
    y_des = double(y_des_sym(t_i));
    
end


