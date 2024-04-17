function [lb, ub] = Lower_Upper_bounds(max_e1, max_e2, max_delta_u, N)
    % Define bounds for the state variables
    xmin = [-max_e1; -Inf; -max_e2; -Inf]; 
    xmax = [max_e1; Inf; max_e2; Inf];
    
    % Repeat bounds for the states across N+1 instances
    lb_state = repmat(xmin, N+1, 1); 
    ub_state = repmat(xmax, N+1, 1); 

    % Define bounds for control inputs
    umin = -max_delta_u;
    umax = max_delta_u;
    
    % Repeat bounds for the control inputs across N instances
    lb_control = repmat(umin, N, 1); 
    ub_control = repmat(umax, N, 1); 

    % Concatenate lower and upper bounds for all decision variables
    % states first then control
    lb = [lb_state ; lb_control];
    ub = [ub_state ; ub_control];
end
