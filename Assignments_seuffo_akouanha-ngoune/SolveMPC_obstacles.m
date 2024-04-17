function[xmeasure,psides_value,psidot_des_value,x_mpc,y_mpc] = SolveMPC_obstacles(Ad,Bd,n,m,x0, ...
                                                            N,x_des_mpc, y_des_mpc, ...
                                                            psidot_des_mpc, ...
                                                            psi_des_mpc, ...
                                                            s_ref,Vx,obstacle_1x, ...
                                                            obstacle_1y,obstacle_2x,obstacle_2y)
        run('num_parameters.m');
        xmeasure = x0;
        % Number of MPC iterations 
        mpciterations = 50;
        delta = 0.09;

        % Set options
        tol_opt = 1e-3;
        % Define your optimization options
        options = optimoptions('quadprog', ...
                               'Algorithm', ...
                               'active-set', ...
                               'Display','none', ...
                               'StepTolerance',1e-3, ...
                               'ConstraintTolerance', tol_opt, ...
                               'TolFun', 1e-2, ...
                                'MaxIter', 30000);

        %options = optimoptions('linprog','Algorithm','dual-simplex');
                          
        % Now, we can proceed with our MPC controller design and simulation.
        
        % Define the cost parameters (adjust the param for the quality of the path
        % planner)
        Q = diag([1e3,1,2e2,3e3]);

        % Input cost matrix
        param_r = 1e2;
        R = param_r * eye(m);
        
        % Terminal state cost matrix 
        Pf = diag([1, 1, 1 ,1]);
        
        % Define initial time
        tmeasure = 0;
        
        %Terminal states 
        xTerm = [0;0;0;0];
        
        % Define initial guess for open-loop input sequence (u0)
        u0 = repmat(ones(m,1), N, 1);
        
        % Define initial guess for open-loop state sequence (x0)
        x0 = repmat(xmeasure, N+1, 1);
    
        % Inequality contraints (H_x*x<= k_x and H_u <= k_u) 
        % Define inequality constraints for lateral deviation (e1) and orientation error (e2)
        Hx = [1, 0, 0, 0;   % Constraint on e1 (upper bound)
              -1, 0, 0, 0;  % Constraint on -e1 (lower bound)
              0, 0, 1, 0;   % Constraint on e2 (upper bound)
              0, 0, -1, 0]; % Constraint on -e2 (lower bound)
        
        kx = [max_e1;      % Upper bound on e1
              max_e1;      % Lower bound on -e1
              max_e2;      % Upper bound on e2
              max_e2];     % Lower bound on -e2
        
        % Define inequality constraints for control input (delta_u)
        Hu = [1;   % Constraint on delta_u (upper bound)
              -1]; % Constraint on -delta_u (upper bound)
        ku = [max_delta_u;  % Upper bound on delta_u
              max_delta_u]; % Upper bound on -delta_u
        
        % ==============================================
        % Implement the MPC iteration
        % ==============================================
        
        % Use the quadprog command to solve the optimization problem
        % quadprog has the following command
        % x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options)
        % x: optimal solution
        % H: quadratic part of the cost
        % f: linear part of the cost
        % A, b:inequality constraints A.x <= b
        % Aeq, beq: equality constraints Aeq.x = beq
        % lu, ub: lower and upper bound constraints
        % x0: initial guess for the optimization
        % options: options for the solver
        % Define Matrices for quadprog using dynamic constraint formulation
        % State constraints (repeat for each time step)
        
        % ==============================================
        % === === === === === === === === === === === ==
        % ==============================================

        Ax = [];
        bx = [];
        for k = 1:N
            Ax = blkdiag(Ax, Hx); % Repeat Hx N times horizontally
            bx = [bx; xTerm]; % Repeat kx N times
        end
        clear('k')

        % Terminal state constraint (for the last state xN)
        Ax = blkdiag(Ax, Hx); % Add terminal state constraint
        bx = [bx; kx]; % Add terminal state constraint
        
        % Input constraints (repeat for each time step)
        Au = [];
        bu = [];
        for k = 1:N
            Au = blkdiag(Au, Hu); % Repeat Hu N times horizontally
            bu = [bu; ku]; % Repeat ku N times
        end
        clear('k')
        
        % Concatenate state and input constraints vertically
        A = blkdiag(Ax, Au);
        b = [bx; bu];
        
        % Equality constraints (dynamics; stacking)
        Aeq = zeros(n*N, n*(N+1) + m*N); 
        beq = zeros(n*N, 1);
        
        for k = 1:N
            % State transition constraints
            Aeq((n*(k-1)+1):(n*k), (n*(k-1)+1):(n*k)) = eye(n);                      % x(k)
            Aeq((n*(k-1)+1):(n*k), (n*k+1):(n*(k+1))) = -Ad;                         % -Ad * x(k+1)
            Aeq((n*(k-1)+1):(n*k), n*(N+1) + (m*(k-1)+1):n*(N+1) + m*k) = -Bd(:,1);  % -Bd(:,1) * u(k)
        end
        
        % Initial condition constraint
        Aeq(n*N+1:n*(N+1), 1:n) = eye(n);
        beq(n*N+1:n*(N+1)) = xmeasure;
         
        % Terminal condition constraint
        Aeq(n*(N+1)+1:n*(N+2), n*N+1:n*(N+1)) = eye(n);
        beq(end-n + 1:end) = xTerm;
        beq = [beq; xTerm];
        
        % Now, let's define the cost function for the quadratic programming problem
        % Define stage cost matrices (repeat for each time step)
        Qstack = [];
        Rstack = [];
        for k = 1:N
            Qstack = blkdiag(Qstack, delta * Q); % State cost at each time step
            Rstack = blkdiag(Rstack, delta * R); % Input cost at each time step
        end
        clear ('k');
        
        % Construct the overall cost matrix
        H = blkdiag(Qstack, Rstack, Pf);
        
        % objective function
        f = zeros(1, n * (N + 1) + m * N );
        
        % bounds for optimization variables
        %[lb , ub] = Lower_Upper_bounds(max_e1, max_e2,max_delta_u, N);
        
        % Print Header
        fprintf('   k  |      u(k)        x(1)        x(2)        x(3)        x(4)     Time \n');
        fprintf('---------------------------------------------------------------------------\n');
        
        % Initialize sum_cost to zero
        sum_cost = 0;
    
        %nitial progression along the path
        s_current = 0;
        
        %Iniitialize the state variables, control input and time vector 
        x=[];
        u=[];
        t=[];
        
        % Compute cost for the current iteration
        current_cost = 0;  % Initialize current_cost to zero
        
        %Storing the list of constraints
        list_max =[];
        list_max2 = [];

        %observation of the cost function 
        previous_cost = Inf;
        broken_ii = 0;

        %obstacles array 
        obstacles = {[obstacle_1x,obstacle_2x],[obstacle_1y,obstacle_2y]};

for ii = 1:mpciterations
    
    % Update s_current based on Vx and delta_t
    % For a more accurate simulation, this might involve integrating over the actual path traveled, considering dynamics.
    s_current = s_current + Vx * tmeasure;
    
    % Ensure s_current does not exceed the total path length
    s_current = min(s_current, s_ref(end));
    
    % Find the closest index in s_ref to the current s_current
    [~, idx] = min(abs(s_ref - s_current));
    
    % Use the psidot_des_mpc and psi_des value at this index for the vehicle dynamics update
    psidot_des_value = psidot_des_mpc(idx+20);
    psides_value = psi_des_mpc(idx+20);
    
    % Update heading 
    heading = xmeasure(3, end) + psides_value;        % Update heading
    
    % Update global position 
    x_mpc = x_des_mpc - xmeasure(1,end)*sin(heading);   % Update global X position
    y_mpc = y_des_mpc + xmeasure(1,end)*cos(heading);   % Update global Y position
    % Gradually tighten the constraint for e1 and e2
    max_e1 = max(max_e1_final, max_e1_initial + (mpciterations - ii) / mpciterations);
    max_e2 = max(max_e2_final, max_e2_initial + (mpciterations - ii) / mpciterations);

    % bounds for optimization variables
    [lb , ub] = Lower_Upper_bounds(max_e1, max_e2,max_delta_u, N);

    % Update constraints based on the current max_e1
    kx = [max_e1;      % Upper bound on e1
          max_e1;      % Lower bound on -e1
          max_e2;      % Upper bound on e2
          max_e2];     % Lower bound on -e2

    % Update based on the proximity with the global position 
    kx = generate_obstacle_constraints(obstacles, x_mpc, y_mpc, min_distance, kx);

    % Since bx depends on kx, and bx is part of b, we need to update bx
    bx = [];
    for k = 1:N
        bx = [bx; kx]; % Repeat kx N times
    end
    
    bx = [bx; kx]; % This includes the terminal constraint for e1 and e2 again
    
    b = [bx; bu]; % Assuming bu is predefined and constant across iterations
    
    % Update constraints (initial constraint) 
    Aeq(n*(N+1)+1:n*(N+2), 1:n) = eye(n); 
    beq(n*N+1:n*(N+1)) = xmeasure;
    
    t_start = tic;

    % Solve optimization problem
    [solutionOL, ~, flag] = quadprog(H, f, A, b, Aeq, beq, lb, ub, [x0; u0],options);
    
    % Check optimization results
    if flag == 0
        warning("Optimization problem not solved, maximum number of iterations reached")
    elseif flag < 0
        error("Optimization problem not feasible")
    end
    
    % Derive the optimal predicted state and input sequence
    x_OL_tilde = solutionOL(1:n*(N+1), 1);
    u_OL_tilde = solutionOL(n*(N+1)+1:end, 1);
    x_OL = reshape(x_OL_tilde, n, N+1);
    u_OL = reshape(u_OL_tilde, m, N);
    
    t_elapsed = toc(t_start);
    % Store closed-loop data
    t = [t, tmeasure];
    x = [x, xmeasure];
    u = [u, u_OL(:,1)];
    
    % Iterate over each time step
    
    current_cost =  x(:, ii)' * Q * x(:, ii) + u(:, ii)' * R * u(:, ii) +  x(:,end)' * Pf * x(:, end);
    

    % Compare the current cost with the previous cost
    if current_cost > previous_cost
        fprintf('Optimization stopped due to an increase in cost at iteration %d.\n', ii);
        broken_ii = ii;
        xmeasure = x(:,broken_ii-1);
        break; % Exit the loop if the current cost has increased
    else
        % Update the previous_cost for the next iteration
        previous_cost = current_cost;
    end
    
    % Update sum_cost
    sum_cost = sum_cost + current_cost;
    
    % Update the closed-loop system
    xmeasure = Ad * xmeasure + Bd * [u_OL(:,1);psidot_des_value];
    tmeasure = tmeasure + delta;

    x_upt = repmat(xmeasure,1,N+1);
    %Add linear terms for state deviation
    f(1:length(xmeasure)) = -Q * xmeasure;        % Add linear terms for state deviation
    f(length(xmeasure)+1:end) = - R * u_OL(:,1);    % Add linear terms for control effort
    
    % Prepare warmstart solution for next time step
    x0 = [x_OL_tilde(n+1:end); zeros(n,1)];
    u0 = [u_OL_tilde(m+1:end); ones(m,1)];
    
    %[ineq,eq,ncalls] = deletionfilter(A,b,Aeq,beq,lb,ub);

    % Print results
    fprintf(' %3d  | %+11.6f %+11.6f %+11.6f %+11.6f %+11.6f  %+6.3f\n', ii, u(end), x(1,end),x(2,end),x(3,end), x(4,end), t_elapsed);
    disp("Integrated Cost in close loop:")
    disp(current_cost)

end 

end