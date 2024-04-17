clc                            
close all                       
clear all                       
%% Parameters 

run ('num_parameters.m');        % Run script to set numerical parameters
run ('parameter_init.m');        % Run script to initialize parameters

%% LQR
xdotPP = @xdotPP;                % Define function handle for state-space system
%% Recalling function

Vx = 30/3.6;
Vdes = velocity_interval(2);

% Compute state matrix A
A = Amatrix(mass,Vx,cf,cr,lf,lr,Iz); 

% Compute input matrix B1
B = B1matrix(mass,cf,lf,Iz);   

% Compute disturbance input matrix B_d
B_d = B_dmatrix(mass,cf,cr,Iz,Vx,lf,lr); 

% Identity matrix for output
C = eye(4); 

% Zeros matrix for direct feedthrough
D = zeros(4,2);

[omega_n, S] = specifications(deltap,deviation,zeta);
%%  CONTROLLER 

% Initial state
x0 = [-2; 0; 0; 0];

% Default plot storage
t_plot = 0:dt:t_end*2;           % Simulation time
heading_plot = 0;                 % Vehicle heading
heading_error = 0;                % Vehicle heading error
slip_angle = 0;                   % Slip angle
course_angle = 0;                 % Course angle
steering_angle = 0;               % Steering angle
pos_des = [0; 0];                 % Desired position
position_global = [0; -2];     % Global position

% Check the reachability
% Controllability matrix
R = [B A*B (A^2)*B (A^3)*B];  

% Rank of controllability matrix
rank_R = rank(R);

if rank_R == length(A)
    disp('FULLY REACHABLE')
else 
    disp('NOT FULLY REACHABLE')
end

xpp= x0;                          % Initialize state variable
%% Controller simulation 

while t <= 2*t_end

    % Update state matrix A based on current velocity
    A1 = Amatrix(mass,Vx,cf,cr,lf,lr,Iz);   

    % Update input matrix B1 based on current velocity
    B1 = B1matrix(mass,cf,lf,Iz);         

    % Update disturbance input matrix B_d based on current velocity
    B_d1 = B_dmatrix(mass,cf,cr,Iz,Vx,lf,lr); 

    % controller gains 
    K = PolePlacement(deltap,omega_n,A1,B1);   
    %K = LQR(A1,B1);                 

    % Compute desired path parameters
    [w, psi_des_t, xdes_t, ydes_t] = Desired_path(Vx, r, t);

    % Define the feedforward term for zero steady-state error
    delta_ff = feedforward(mass, Vx, lf, lr, cf, cr, r, K);

    % Simulate system dynamics
    [tsol, xsolpp] = ode45(@(t,x) xdotPP(A1,B1,x,B_d1,w,K,delta_ff), [t t+dt], xpp(:,end)); 

    % Update state variable
    xpp = [xpp xsolpp(end,:)'];                
    % UPDATES 

    % Update heading 
    heading = xpp(3, end) + psi_des_t;

    % Store heading data
    heading_plot = [heading_plot xpp(3, end)'];

    % Update global position 
    % Update global X position
    X1 = xdes_t - xpp(1,end)*sin(heading);

    % Update global Y position
    Y1 = ydes_t + xpp(1,end)*cos(heading);

    % Store desired position data
    pos_des = [pos_des [xdes_t ydes_t]']; 

    % Store global position data
    position_global = [position_global [X1 Y1]'];

    % Update slip angle
    vehicle_slip = (1/Vx)*xpp(2, end) - xpp(3, end); % Compute slip angle

    % Store slip angle data
    slip = [slip vehicle_slip']; 


    % Update course angle
    course_t = heading + vehicle_slip;  
    
    % Store course angle data
    course_angle = [course_angle course_t'];  

    % Update steering angle 
    u = (-K*xpp(:,end)) + delta_ff;   

    % Store steering angle data
    steering_angle = [steering_angle u']; 

    % Expected heading error
    e2 = (-(lr/r)) + ((lf/(2* cr * (lf + lr))) * ((mass*Vx^2)/r));

    % Store heading error data
    heading_error = [heading_error e2'];   

    % Increment time
    t = t + dt;                               
    if t >= 10

        % Radius for a circular path after 10 seconds
        r = 50;   
    else

        % Infinite radius for a straight line for 10 seconds
        r = inf;   
    end  
end
%% Plot the result

% Re-fetch the time simulation
t_plot = 0:dt:2*t_end;

% Plot heading
f1=figure(1);
subplot(3,2,1);
plot(t_plot, heading_plot,'r','LineWidth', 1.5);
title('Heading');
xlabel('Time (s)');
ylabel('Heading (rad)');

% Plot heading error
subplot(3,2,2);
plot(t_plot, heading_error,'g','LineWidth', 1.5);
title('Heading Error');
xlabel('Time (s)');
ylabel('Heading Error (rad)');

% Plot slip angle
subplot(3,2,3);
plot(t_plot, slip,'b','LineWidth', 1.5);
title('Slip Angle');
xlabel('Time (s)');
ylabel('Slip Angle (rad)');

% Plot course angle
subplot(3,2,4);
plot(t_plot, course_angle,'c','LineWidth', 1.5);
title('Course Angle');
xlabel('Time (s)');
ylabel('Course Angle (rad)');

% Plot steering angle
subplot(3,2,5);
plot(t_plot, steering_angle,'m','LineWidth', 1.5);
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (rad)');

% Plot global position
subplot(3,2,6);
plot(position_global(1,:),position_global(2,:), 'b','LineWidth', 1.5);
hold on;
plot(pos_des(1,:), pos_des(2,:), 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on;


f2= figure(2);
plot(position_global(1,:),position_global(2,:), 'b','LineWidth', 1.5);
hold on;
plot(pos_des(1,:), pos_des(2,:), 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on